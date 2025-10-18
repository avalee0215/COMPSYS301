/* ========================= main.c (with PI steering) =========================
 * - S1/S2 still request blocking timed pivots (Directions_* module)
 * - Straight segments now use a PI controller on S4..S6 to keep the line centered
 * - Sensors, motor APIs, and ISR structure otherwise unchanged
 * ============================================================================ */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>

#include <sensors.h>     // Sensor_ComputePeakToPeak()
#include "motor_s.h"     // set_motors_*, motor_enable  (unchanged API)
#include "directions.h"  // Directions_* turning module

/* ===== Loop pacing (keep your 8 ms) ===== */
#define LOOP_DT_MS               8u
#define DT_S   ( (float)LOOP_DT_MS / 1000.0f )

/* ===== Cruise speed / distance target (kept) ===== */
#define VMAX_CONST_MM_S        800
#define SPEED_FRAC_PERCENT      25
#define V_CRUISE_MM_S  ((int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100)
#define TARGET_DIST_MM        20000

/* ===== Encoder → mm conversion (kept from your code) ===== */
#define QD_SAMPLE_MS             5u
#define CPR_OUTSHAFT           228u
#define R_MM                    34
#define PI_X1000              3142
#define PERIM_MM_X1000   ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )
#define CALIB_DIST_X1000     1500
#define APPLY_CALIB_DIST(x)  ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000 + 500)/1000) )

/* ===== S1/S2 relaxed detection (kept) ===== */
#define S_MINC_COUNTS            2u
#define S_MAXC_COUNTS          450u
#define S_HYST_COUNTS           16u

/* ===== Turn request filtering (kept) ===== */
#define TURN_DEBOUNCE_TICKS       5u
#define CLEAR_ARM_TICKS           4u

/* ===== Local sensor flags (used for S1/S2 edge) ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;

/* ===== Global state (kept) ===== */
static volatile uint8_t g_direction = 0;   /* 0=straight, 1=left, 2=right */
static volatile uint8_t g_stop_now  = 0;
static volatile int32_t g_dist_mm   = 0;

/* -------------------------------
 * 5 ms Timer ISR: accumulate distance (kept)
 * ------------------------------- */
CY_ISR(isr_qd_Handler)
{
    int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
    int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

    int32_t d1 = raw1, d2 = raw2;
    int32_t a1 = (d1 >= 0) ? d1 : -d1;
    int32_t a2 = (d2 >= 0) ? d2 : -d2;
    int32_t davg_abs  = (a1 + a2) / 2;
    int32_t davg_sign = ((d1 + d2) >= 0) ? +1 : -1;

    int64_t num_abs = (int64_t)davg_abs * MM_PER_COUNT_X1000;
    int32_t dmm_abs = (int32_t)((num_abs + 500) / 1000);
    int32_t dmm_signed = APPLY_CALIB_DIST(dmm_abs) * davg_sign;

    g_dist_mm += dmm_signed;
    (void)Timer_QD_ReadStatusRegister();     // clear TC
}

/* ----------------------------------------------------------------
 * Utility: saturate and normalize peak-to-peak to [0..1] emphasis
 *  - below S_MINC -> 0, above S_MAXC -> 1, linear in-between
 * ---------------------------------------------------------------- */
static inline float norm01_from_pp(uint16_t pp)
{
    if (pp <= S_MINC_COUNTS) return 0.0f;
    if (pp >= S_MAXC_COUNTS) return 1.0f;
    return (float)(pp - S_MINC_COUNTS) / (float)(S_MAXC_COUNTS - S_MINC_COUNTS);
}

/* -------------------------------------------------------------
 * Read sensors and (maybe) request a turn based on S1 / S2
 *  - identical S1/S2 logic & flags as before (relaxed + hysteresis)
 *  - still sets sen3..sen6 boolean bands for quick checks
 * ------------------------------------------------------------- */
static void light_sensors_update_and_maybe_request_turn(
    /* also return the raw PP values for S4..S6 for PI */ 
    uint16_t* V4_pp, uint16_t* V5_pp, uint16_t* V6_pp)
{
    /* 1) Read ADC peak-to-peak */
    uint16_t V1 = Sensor_ComputePeakToPeak(0);
    uint16_t V2 = Sensor_ComputePeakToPeak(1);
    uint16_t V3 = Sensor_ComputePeakToPeak(2);
    uint16_t V4 = Sensor_ComputePeakToPeak(3);
    uint16_t V5 = Sensor_ComputePeakToPeak(4);
    uint16_t V6 = Sensor_ComputePeakToPeak(5);

    if (V4_pp) *V4_pp = V4;
    if (V5_pp) *V5_pp = V5;
    if (V6_pp) *V6_pp = V6;

    /* 2) S1/S2 relaxed + hysteresis (kept) */
    static uint8_t s1 = 0, s2 = 0;
    uint8_t s1_enter = (V1 > S_MINC_COUNTS && V1 < S_MAXC_COUNTS);
    uint8_t s2_enter = (V2 > S_MINC_COUNTS && V2 < S_MAXC_COUNTS);
    uint8_t s1_exit  = (V1 > (S_MAXC_COUNTS + S_HYST_COUNTS)) ||
                       ((V1 + S_HYST_COUNTS) < S_MINC_COUNTS);
    uint8_t s2_exit  = (V2 > (S_MAXC_COUNTS + S_HYST_COUNTS)) ||
                       ((V2 + S_HYST_COUNTS) < S_MINC_COUNTS);

    if (!s1) s1 = s1_enter ? 1u : 0u; else s1 = s1_exit ? 0u : 1u;
    if (!s2) s2 = s2_enter ? 1u : 0u; else s2 = s2_exit ? 0u : 1u;

    sen1_on_line = s1;
    sen2_on_line = s2;

    /* 3) S3..S6 simple band booleans (kept) */
    sen3_on_line = (V3 > 10 && V3 < 100) ? 1u : 0u;
    sen4_on_line = (V4 > 10 && V4 < 100) ? 1u : 0u;
    sen5_on_line = (V5 > 10 && V5 < 100) ? 1u : 0u;
    sen6_on_line = (V6 > 10 && V6 < 100) ? 1u : 0u;

    /* 4) Turn request on S1/S2: debounce + latch + must-clear-first */
    static uint8_t debL = 0, debR = 0;
    static uint8_t latched_any = 0;
    static uint8_t armed = 0;
    static uint8_t clear_cnt = 0;

    uint8_t left_now  = sen1_on_line;
    uint8_t right_now = sen2_on_line;
    uint8_t both_clear_now = (uint8_t)(!left_now && !right_now);

    if (both_clear_now) {
        if (clear_cnt < CLEAR_ARM_TICKS) clear_cnt++;
        if (clear_cnt >= CLEAR_ARM_TICKS) armed = 1u;
    } else {
        clear_cnt = 0;
    }

    debL = left_now  ? (uint8_t)(debL + 1u) : 0u;
    debR = right_now ? (uint8_t)(debR + 1u) : 0u;

    uint8_t left_stable  = (debL >= TURN_DEBOUNCE_TICKS);
    uint8_t right_stable = (debR >= TURN_DEBOUNCE_TICKS);
    uint8_t any_now      = (uint8_t)(left_stable | right_stable);

    if (g_direction == 0u){
        if (armed && !latched_any && any_now){
            g_direction = left_stable ? 1u : 2u;   // 1=LEFT (S1), 2=RIGHT (S2)
            latched_any = 1u;
            armed = 0u;
        } else if (!any_now){
            latched_any = 0u;
        }
    } else {
        latched_any = 1u;
    }
}

/* ================= PI Controller (steer) =================
 * error e = weighted position from S4..S6, where
 *   S4 = -1, S5 = 0, S6 = +1 (weights by normalized contrast)
 * u = Kp*e + Ki*integral(e)
 * Output u is clamped to [-STEER_MAX..+STEER_MAX] (in duty%)
 * Integral anti-windup: freeze integration when output is clamped
 * ======================================================== */

/* Tunables (start conservative; increase Kp first, then Ki) */
#define STEER_MAX        18     /* max ± steer duty applied */
#define KP               14.0f  /* proportional gain (duty per unit error) */
#define KI               2.0f   /* integral gain (duty per unit error per second) */
#define INT_LIM          30.0f  /* clamp for integral state (to avoid windup burst) */
#define LOSS_TIMEOUT_T   0.25f  /* s without valid line before zeroing integral */

typedef struct {
    float i;           /* integral state */
    float u;           /* last command */
    float t_loss;      /* time since last “valid” reading */
} pi_t;

static inline float _clampf(float x, float lo, float hi){ return (x<lo?lo:(x>hi?hi:x)); }

/* returns steer command in duty% (±) */
static int pi_step(pi_t* pi, uint16_t V4_pp, uint16_t V5_pp, uint16_t V6_pp)
{
    /* Normalize contrasts 0..1 using your S_MINC/S_MAXC band */
    float c4 = norm01_from_pp(V4_pp);
    float c5 = norm01_from_pp(V5_pp);
    float c6 = norm01_from_pp(V6_pp);

    /* If all nearly zero, line is “lost” */
    float sum = c4 + c5 + c6;
    bool valid = (sum > 0.08f);  /* small epsilon */

    /* Weighted position: left=-1, center=0, right=+1 */
    float pos = 0.0f;
    if (valid) pos = (-1.0f * c4 + 0.0f * c5 + 1.0f * c6) / sum;

    /* Error: positive means line is to the right → steer right (speed up right wheel) */
    float e = pos;

    /* Loss handling: if invalid for a short time, keep last u; after timeout, decay integral */
    if (!valid) {
        pi->t_loss += DT_S;
        if (pi->t_loss >= LOSS_TIMEOUT_T) {
            /* gently bleed the integrator to zero when lost for a while */
            const float bleed = 0.92f;
            pi->i *= bleed;
        }
        /* hold last u (don’t integrate) */
        return (int)_clampf(pi->u, -STEER_MAX, STEER_MAX);
    }
    pi->t_loss = 0.0f;

    /* Integrate with clamp */
    float i_next = pi->i + e * DT_S;
    i_next = _clampf(i_next, -INT_LIM, +INT_LIM);

    /* Raw (unclamped) output */
    float u_raw = KP * e + KI * i_next;

    /* Clamp output, and do simple conditional anti-windup:
       only accept the new integral if we are not saturating “against” the error */
    float u = _clampf(u_raw, -((float)STEER_MAX), +((float)STEER_MAX));

    bool sat_hi = (u >=  (float)STEER_MAX - 1e-3f);
    bool sat_lo = (u <= -(float)STEER_MAX + 1e-3f);

    /* If saturated and the new integral would push further into saturation, don’t commit it */
    if ( (sat_hi && (KI * i_next > KI * pi->i)) ||
         (sat_lo && (KI * i_next < KI * pi->i)) ) {
        /* keep previous integral */
    } else {
        pi->i = i_next;
    }

    pi->u = u;
    return (int)(u + (u>=0?0.5f:-0.5f));  /* round to int */
}

int main(void)
{
    CyGlobalIntEnable;

    /* ADC for sensors */
    ADC_Start();

    /* Encoders + 5 ms tick (distance only) */
    Clock_QENC_Start();
    QuadDec_M1_Start(); QuadDec_M2_Start();
    QuadDec_M1_SetCounter(0); QuadDec_M2_SetCounter(0);
    Clock_QD_Start();
    Timer_QD_Start();                  // 5 ms period in TopDesign
    isr_qd_StartEx(isr_qd_Handler);

    /* PWM & motor driver */
    Clock_PWM_Start();
    PWM_1_Start(); PWM_2_Start();
    PWM_1_WritePeriod(PWM_PERIOD);
    PWM_2_WritePeriod(PWM_PERIOD);
    motor_enable(0u, 0u);
    set_motors_symmetric(0);

    /* Directions module (for your blocking pivots) */
    Directions_Init();
    g_direction = 0u;

    /* Feed-forward cruise duty (kept) */
    int center_duty_est = (int)((V_CRUISE_MM_S * 100) / VMAX_CONST_MM_S);
    if (center_duty_est < 0) center_duty_est = 0;
    if (center_duty_est > 100) center_duty_est = 100;

    /* PI state */
    pi_t pi = { .i = 0.0f, .u = 0.0f, .t_loss = 0.0f };

    for(;;){
        /* Distance stop */
        g_stop_now = (g_dist_mm >= TARGET_DIST_MM) ? 1u : 0u;
        if (g_stop_now) {
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            g_direction = 0u;
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Read sensors + maybe request turn */
        uint16_t V4_pp=0, V5_pp=0, V6_pp=0;
        light_sensors_update_and_maybe_request_turn(&V4_pp, &V5_pp, &V6_pp);

        /* Handle a requested pivot (unchanged) */
        if (g_direction == 1u || g_direction == 2u){
            Directions_Handle(&g_direction);
            /* after a turn, reset integrator so we don’t carry a bias */
            pi.i = 0.0f; pi.u = 0.0f; pi.t_loss = 0.0f;
            CyDelay(LOOP_DT_MS);
            continue;
        }

        /* Straight run with PI steering */
        int steer = pi_step(&pi, V4_pp, V5_pp, V6_pp);
        set_motors_with_trim_and_steer(center_duty_est, steer);

        CyDelay(LOOP_DT_MS);
    }
}

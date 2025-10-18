/* ========================= main.c =========================
 * - sensors.* UNCHANGED (uses Sensor_ComputePeakToPeak)
 * - directions.* used for timed pivots on demand
 * - motor: uses new motor.h / motor.c (set_motors_* API)
 *
 * Behavior:
 *   - S1 (left)  → LEFT turn (g_direction=1)
 *   - S2 (right) → RIGHT turn (g_direction=2)
 *   - Otherwise  → straight running with line trim (S4–S6)
 *
 * Improvements kept:
 *   - S1/S2: widened band + hysteresis (less-dark shadows OK)
 *   - Edge-trigger + latch, must-clear-first arming
 * ========================================================= */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>

#include <sensors.h>     // Sensor_ComputePeakToPeak()
#include "motor_s.h"       // new motor API (set_motors_*, motor_enable)
#include "directions.h"  // Directions_* turning module

/* ===== Loop pacing (optional) ===== */
#define LOOP_DT_MS               8u

/* ===== Speed / distance (simple cruise) ===== */
#define VMAX_CONST_MM_S        800
#define SPEED_FRAC_PERCENT      25
#define V_CRUISE_MM_S  ((int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100)
#define TARGET_DIST_MM        1100

/* ===== Encoder → mm conversion (same as 이전 코드 값) ===== */
#define QD_SAMPLE_MS             5u
#define CPR_OUTSHAFT           228u
#define R_MM                    34
#define PI_X1000              3142
#define PERIM_MM_X1000   ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )
#define CALIB_DIST_X1000     1500     /* 현장 보정값 유지 */
#define APPLY_CALIB_DIST(x)  ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000 + 500)/1000) )

/* ===== S1/S2 relaxed detection (band + hysteresis) ===== */
#define S_MINC_COUNTS            2u     // was 10
#define S_MAXC_COUNTS          450u     // was 100
#define S_HYST_COUNTS           16u     // enter/exit gap to prevent flicker

/* ===== Turn request filtering ===== */
#define TURN_DEBOUNCE_TICKS       5u    // consecutive ticks to confirm S1/S2
#define CLEAR_ARM_TICKS           4u    // both S1 & S2 clear briefly before first turn

/* ===== Local sensor flags (used for steering & decisions) ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;

/* Global direction flag: 0=straight, 1=left, 2=right */
static volatile uint8_t g_direction = 0;

/* Stop is the 1st thing to think */
static volatile uint8_t g_stop_now = 0;

/* ----- minimal odometry (Δcount → mm 누적) ----- */
static volatile int32_t g_dist_mm = 0;

/* ------------------------------------------------------------------
 * Steering bias provider (unchanged style; uses S4–S6 for trim)
 * ------------------------------------------------------------------ */
static inline int scaled_steer_step(int base_duty) {
    const int STEER_STEP = 1, MIN_STEER_STEP = 1;
    int pct = (base_duty >= 0) ? base_duty : -base_duty;
    int step = (pct < 50) ? (STEER_STEP * (pct + 10) + 59) / 60 : STEER_STEP;
    if (step < MIN_STEER_STEP) step = MIN_STEER_STEP;
    return step;
}
static int steer_bias_provider(int center_duty){
    int small = scaled_steer_step(center_duty);
    int big   = (small*3+1)/2;

    if (sen6_on_line && sen4_on_line && !sen5_on_line) return +small;
    if (sen6_on_line && sen5_on_line && !sen4_on_line) return -small;
    if (!sen6_on_line && sen4_on_line && !sen5_on_line) return +big;
    if (!sen6_on_line && sen5_on_line && !sen4_on_line) return -big;
    return 0;
}

/* -------------------------------
 * 5 ms Timer ISR: accumulate distance
 * ------------------------------- */
CY_ISR(isr_qd_Handler)
{
    int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
    int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

    /* direction-safe distance: abs(avg) * sign(sum) → mm */
    int32_t d1 = raw1, d2 = raw2; // (ENC_SIGN가 동일하면 여기서 부호 반영 가능)
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

/* -------------------------------------------------------------
 * Read sensors and (maybe) request a turn based on S1 / S2 only
 *  - S1/S2: relaxed band + hysteresis so they don’t need “dark-dark”
 *  - S3..S6: keep the simple band (10..100) for steering trim
 * ------------------------------------------------------------- */
static void light_sensors_update_and_maybe_request_turn(void)
{
    /* 1) Read ADC peak-to-peak (unchanged calls into sensors.c) */
    uint16_t V1 = Sensor_ComputePeakToPeak(0);
    uint16_t V2 = Sensor_ComputePeakToPeak(1);
    uint16_t V3 = Sensor_ComputePeakToPeak(2);
    uint16_t V4 = Sensor_ComputePeakToPeak(3);
    uint16_t V5 = Sensor_ComputePeakToPeak(4);
    uint16_t V6 = Sensor_ComputePeakToPeak(5);

    /* 2) S1/S2 relaxed + hysteresis */
    static uint8_t s1 = 0, s2 = 0;  // hysteresis memory (0=off, 1=on)

    uint8_t s1_enter = (V1 > S_MINC_COUNTS && V1 < S_MAXC_COUNTS);
    uint8_t s2_enter = (V2 > S_MINC_COUNTS && V2 < S_MAXC_COUNTS);

    uint8_t s1_exit  = (V1 > (S_MAXC_COUNTS + S_HYST_COUNTS)) ||
                       ((V1 + S_HYST_COUNTS) < S_MINC_COUNTS);
    uint8_t s2_exit  = (V2 > (S_MAXC_COUNTS + S_HYST_COUNTS)) ||
                       ((V2 + S_HYST_COUNTS) < S_MINC_COUNTS);

    if (!s1) s1 = s1_enter ? 1u : 0u;
    else     s1 = s1_exit  ? 0u : 1u;

    if (!s2) s2 = s2_enter ? 1u : 0u;
    else     s2 = s2_exit  ? 0u : 1u;

    sen1_on_line = s1;
    sen2_on_line = s2;

    /* 3) S3..S6 keep simple band (adjust if you want later) */
    sen3_on_line = (V3 > 10 && V3 < 100) ? 1u : 0u;
    sen4_on_line = (V4 > 10 && V4 < 100) ? 1u : 0u;
    sen5_on_line = (V5 > 10 && V5 < 100) ? 1u : 0u;
    sen6_on_line = (V6 > 10 && V6 < 100) ? 1u : 0u;

    /* 4) Turn request on S1/S2: debounce + latch + must-clear-first arming */
    static uint8_t debL = 0, debR = 0;     // debounce counters
    static uint8_t latched_any = 0;        // blocks re-trigger until clear
    static uint8_t armed = 0;              // allow turns only after clear period
    static uint8_t clear_cnt = 0;

    uint8_t left_now  = sen1_on_line;
    uint8_t right_now = sen2_on_line;
    uint8_t both_clear_now = (uint8_t)(!left_now && !right_now);

    // must-clear-first
    if (both_clear_now) {
        if (clear_cnt < CLEAR_ARM_TICKS) clear_cnt++;
        if (clear_cnt >= CLEAR_ARM_TICKS) armed = 1u;
    } else {
        clear_cnt = 0;
    }

    // debounce each side
    debL = left_now  ? (uint8_t)(debL + 1u) : 0u;
    debR = right_now ? (uint8_t)(debR + 1u) : 0u;

    uint8_t left_stable  = (debL >= TURN_DEBOUNCE_TICKS);
    uint8_t right_stable = (debR >= TURN_DEBOUNCE_TICKS);
    uint8_t any_now      = (uint8_t)(left_stable | right_stable);

    if (g_direction == 0u){
        if (armed && !latched_any && any_now){
            g_direction = left_stable ? 1u : 2u;   // 1=LEFT (S1), 2=RIGHT (S2)
            latched_any = 1u;
            armed = 0u;                            // require clear again before next turn
        } else if (!any_now){
            latched_any = 0u;                      // re-arm latch when clear
        }
    } else {
        latched_any = 1u;                          // turning/queued; keep latched
    }
}

/* ============================= main ============================= */
int main(void)
{
    CyGlobalIntEnable;

    /* ADC for sensors */
    ADC_Start();

    /* Encoders + 5 ms tick (distance only; speed 제어는 단순 듀티) */
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
    motor_enable(0u, 0u);              // enable both
    set_motors_symmetric(0);

    /* Directions module */
    Directions_Init();
    g_direction = 0u;                  // clean start: straight mode

    /* Simple cruise center duty from target speed (feed-forward only) */
    /*  v_target / VMAX ~= duty%, deadband는 motor.c 내부에서 처리됨 */
    int center_duty_est = (int)((V_CRUISE_MM_S * 100) / VMAX_CONST_MM_S);
    if (center_duty_est < 0) center_duty_est = 0;
    if (center_duty_est > 100) center_duty_est = 100;

    for(;;){
        /* FIRST THING: stoping */
        g_stop_now = (g_dist_mm >= TARGET_DIST_MM) ? 1u : 0u;
        if (g_stop_now) {
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            g_direction = 0u;          
            CyDelay(LOOP_DT_MS);
            continue;
}
        /* A) Read sensors, possibly request a turn on S1/S2 */
        light_sensors_update_and_maybe_request_turn();

        /* B) If a turn was requested, perform it (blocking) and reset to 0 */
        if (g_direction == 1u || g_direction == 2u){
            Directions_Handle(&g_direction);   // pivot, then set back to 0
            CyDelay(LOOP_DT_MS);               // skip this straight tick
            continue;
        }

        /* C) Straight run:
         *    - stop if distance >= TARGET_DIST_MM
         *    - else apply small steering (틱틱) around center duty
         */
        if (g_dist_mm >= TARGET_DIST_MM){
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            CyDelay(LOOP_DT_MS);
            continue;
        }

        int steer = steer_bias_provider(center_duty_est);
        set_motors_with_trim_and_steer(center_duty_est, steer);

        /* D) Optional pacing */
        CyDelay(LOOP_DT_MS);
    }
}

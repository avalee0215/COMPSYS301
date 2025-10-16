/* ===========================================
 * COMPSYS301 — v4 (5ms loop, safer stop)
 *  - FF + PI + Adaptive Brake
 *  - Light Sensor TILT (NO_ACCUM)
 *  - START: Encoder bi-directional balance (+ RIGHT_TRIM in START)
 *  - RUN: first 300ms encoder assist if sensors neutral
 *  - Distance accumulation: direction-safe (abs*sign)
 *  - Stop trigger: bias(−), speed-slope(−), hardcut guard
 * =========================================== */

#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <sensors.h>

/* ===== User config ===== */
#define VMAX_CONST_MM_S        800
#define SPEED_FRAC_PERCENT     25
#define RIGHT_TRIM_PERCENT     -8      /* 음수면 오른쪽 강화 */

/* ===== Distance goal ===== */
#define TARGET_DIST_MM         1100

/* ===== PWM / Sample ===== */
#define PWM_PERIOD             200u
#define QD_SAMPLE_MS           5u
#define CPR_OUTSHAFT           228u

/* ===== Wheel geometry ===== */
#define R_MM                   34
#define PI_X1000               3142
#define PERIM_MM_X1000         ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )

/* ===== Calibration ===== */
#define CALIB_SPEED_X1000      2167
#define CALIB_DIST_X1000       1500      /* 현장 값으로 조정 권장 */
#define APPLY_CALIB_SPEED(x)   ( (int32_t)(((int64_t)(x) * CALIB_SPEED_X1000 + 500)/1000) )
#define APPLY_CALIB_DIST(x)    ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000  + 500)/1000) )

/* ===== Polarity ===== */
#define RIGHT_MOTOR_SIGN       (-1)  /* M1 = Right */
#define LEFT_MOTOR_SIGN        (+1)  /* M2 = Left  */
#define ENC1_SIGN              (-1)
#define ENC2_SIGN              (-1)

/* ===== Safety (hard, ms-based) ===== */
#define MAX_TRAVEL_MM_SAFE     (TARGET_DIST_MM + 200)
#define MAX_RUN_TIME_MS        25000u
static uint32_t g_run_ms = 0;

/* ===== Adaptive hard-stop tuning ===== */
#define V_STOP_THRESH_MM_S     80
#define BRAKE_OFFSET_BASE_MM   40
#define BRAKE_OFFSET_PER_MM_S  0.22f
#define BRAKE_OFFSET_MIN_MM    20
#define BRAKE_OFFSET_MAX_MM    200
#define BRAKE_DUTY_MIN         30
#define BRAKE_DUTY_PER_MM_S    0.035f
#define BRAKE_DUTY_MAX         55
#define BRAKE_MAX_MS           800u

/* 🔧 Rollback guard */
#define HOLD_FWD_DUTY          5
#define HOLD_TICKS_MAX         4        /* 4×5ms = 20 ms */

/* ===== Control gains (5ms 기준) ===== */
#define KP_x1000               6
#define KI_x1000               200      /* Σ(e*Δt_ms) */
#define ICLAMP                 2000000
#define DUTY_SLEW_PER_SEC      120

/* FF & deadband */
#define DUTY_DEADBAND_PERCENT  12
#define FF_GAIN_x1000          900
#define FF_MAX_DUTY            95

/* ===== Filtering ===== */
static int32_t g_v_filt = 0;
#define VFILT_DEN              3

/* ===== STOP trigger shaping ===== */
#define STOP_TRIGGER_BIAS_MM        (-150)     /* 음수 = 더 일찍 브레이크 */
#define STOP_TRIGGER_SLOPE_PER_V    (-0.040f)  /* (mm per mm/s), 음수 */

/* ===== Shared (ISR->main) ===== */
static volatile uint16_t g_tick = 0;
static volatile int32_t  g_c1 = 0, g_c2 = 0;
static volatile uint8_t  g_ctl_pending = 0;

/* Distance (mm) */
static volatile int32_t  g_dist_mm = 0;

/* ===== Cruise state ===== */
static int               g_duty_cmd      = 50;
static int32_t           g_v_cruise_mm_s = 0;
static int32_t           g_integ         = 0;   /* Σ(e * Δt_ms) */

/* ===== Mode machine ===== */
typedef enum { MODE_START=0, MODE_RUN, MODE_BRAKE, MODE_DONE } run_mode_t;
static volatile run_mode_t g_mode = MODE_START;
static uint16_t g_brake_ms = 0;

/* ===== Motor helpers ===== */
static inline int clamp100(int x){ if(x>100) return 100; if(x<-100) return -100; return x; }
static inline uint16 duty_to_compare(int s)
{
    if (s < -100) s = -100;
    if (s >  100) s =  100;
    return (uint16)((PWM_PERIOD/2) + ((int32)PWM_PERIOD * s)/200);
}
static inline void set_motors_symmetric(int duty)
{
    duty = clamp100(duty);
    PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty));
    PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty));
}
static inline int apply_right_trim(int duty)
{
    int32_t scaled = ((int32_t)duty * (100 - RIGHT_TRIM_PERCENT)) / 100;
    return (int)scaled;
}

/* ==== Per-wheel 최소 듀티 ==== */
#define MIN_WHEEL_DUTY_R   15
#define MIN_WHEEL_DUTY_L   15

static inline void set_motors_with_trim_and_steer(int duty_center, int steer)
{
    int duty_right = clamp100(duty_center + steer);
    int duty_left  = clamp100(duty_center - steer);

    duty_right = clamp100(apply_right_trim(duty_right));

    if (duty_center > 0) {
        if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
        if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;
    }

    PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty_right));
    PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty_left ));
}

/* ===== IO helpers ===== */
static inline void motor_enable(uint8 m1_disable, uint8 m2_disable)
{
    uint8 v = 0;
    if (m1_disable) v |= 0x01;  /* bit0 -> M1_D1 (HIGH=disable) */
    if (m2_disable) v |= 0x02;  /* bit1 -> M2_D1 (HIGH=disable) */
    CONTROL_Write(v);
}

/* ===== Math helpers ===== */
static inline int32_t mm_s_from_counts(int32_t d1, int32_t d2)
{
    int32_t davg = (d1 + d2) / 2;
    int32_t v_mm_s_est = (int32_t)(((int64_t)davg * MM_PER_COUNT_X1000) / (int32_t)QD_SAMPLE_MS);
    return APPLY_CALIB_SPEED(v_mm_s_est);
}
static inline int duty_ff_from_target_mm_s(int32_t v_target)
{
    if (v_target <= 0) return 0;
    int32_t duty_lin = (int32_t)((int64_t)v_target * FF_GAIN_x1000 / VMAX_CONST_MM_S);
    int duty = (int)(duty_lin / 10);
    if (duty > 0 && duty < DUTY_DEADBAND_PERCENT) duty = DUTY_DEADBAND_PERCENT;
    if (duty > FF_MAX_DUTY) duty = FF_MAX_DUTY;
    return duty;
}

/* === adaptive brake helpers === */
static inline int dyn_brake_duty(int32_t v_mm_s_filt){
    int32_t spd = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    float d = (float)BRAKE_DUTY_MIN + BRAKE_DUTY_PER_MM_S * (float)spd;
    if (d > BRAKE_DUTY_MAX) d = BRAKE_DUTY_MAX;
    if (d < 0) d = 0;
    return (int)(d + 0.5f);
}

/* ===== Light sensors ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;
static uint8_t g_direction = 0; /* 1=left, 2=right */

static inline int scaled_steer_step(int base_duty)
{
    const int STEER_STEP = 1;
    const int MIN_STEER_STEP = 1;
    int pct = (base_duty >= 0) ? base_duty : -base_duty;
    int step;
    if (pct < 50) step = (STEER_STEP * (pct + 10) + 59) / 60;
    else          step = STEER_STEP;
    if (step < MIN_STEER_STEP) step = MIN_STEER_STEP;
    return step;
}

static void light_sensors_update(void)
{
    uint16_t V1 = Sensor_ComputePeakToPeak(0);
    uint16_t V2 = Sensor_ComputePeakToPeak(1);
    uint16_t V3 = Sensor_ComputePeakToPeak(2);
    uint16_t V4 = Sensor_ComputePeakToPeak(3);
    uint16_t V5 = Sensor_ComputePeakToPeak(4);
    uint16_t V6 = Sensor_ComputePeakToPeak(5);

    sen1_on_line = (V1 > 10 && V1 < 100);
    sen2_on_line = (V2 > 10 && V2 < 100);
    sen3_on_line = (V3 > 10 && V3 < 100);
    sen4_on_line = (V4 > 10 && V4 < 100);
    sen5_on_line = (V5 > 10 && V5 < 100);
    sen6_on_line = (V6 > 10 && V6 < 100);

    if (sen1_on_line || sen4_on_line) g_direction = 1;
    else if (sen2_on_line || sen5_on_line) g_direction = 2;
}

/* ===== TILT (NO_ACCUM) ===== */
static inline int compute_tilt_bias_inst(void)
{
    int small = scaled_steer_step(g_duty_cmd);
    int big   = (small*3+1)/2;

    if (sen6_on_line && sen4_on_line && !sen5_on_line) return +small;
    if (sen6_on_line && sen5_on_line && !sen4_on_line) return -small;
    if (!sen6_on_line && sen4_on_line && !sen5_on_line) return +big;
    if (!sen6_on_line && sen5_on_line && !sen4_on_line) return -big;

    return 0;
}

/* ===== START: Encoder-based bi-directional balance ===== */
#define START_TOTAL_MS             500
#define START_INIT_DUTY            18
#define START_MAX_DUTY             36
#define START_RAMP_PER_SEC         60     /* 1초에 듀티 +60 */

#define START_BAL_K                2      /* Δcount 게인(1~3) */
#define START_BAL_MAX_STEER        6      /* ±최대 보정 듀티 */

/* (선택) 왼쪽 감쇠 비활성화 */
#define START_LEFT_DAMP_ENABLE        0
#define START_LEFT_DAMP_PCT_BASE      0
#define START_LEFT_DAMP_PCT_PER_DIFF  0
#define START_LEFT_DAMP_PCT_MAX       0
#define START_LEFT_DAMP_MS            0

static uint16_t g_start_elapsed_ms = 0;
static int      g_start_center = START_INIT_DUTY;

/* ===== STOP-FIX ===== */
#define A_DECEL_MM_S2             3400
static inline int32_t stop_dist_mm(int32_t v_mm_s_filt){
    int64_t v = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    int64_t d = (A_DECEL_MM_S2 > 0) ? (v * v) / (2 * (int64_t)A_DECEL_MM_S2) : 0;
    if (d < 20) d = 20; if (d > 250) d = 250;
    return (int32_t)d;
}

/* ===== RUN early encoder assist ===== */
static uint16_t run_elapsed_ms = 0;
#define RUN_ENCODER_ASSIST_MS    300
#define RUN_ENCODER_ASSIST_MAX   4   /* ±4 듀티 */

/* ===== Timer ISR (5 ms) ===== */
CY_ISR(isr_qd_Handler)
{
    g_tick++;

    int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
    int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

    int32_t d1 = ENC1_SIGN * raw1;
    int32_t d2 = ENC2_SIGN * raw2;

    g_c1 = d1;
    g_c2 = d2;

    /* --- Distance accumulation: direction-safe (abs * sign of sum) --- */
    int32_t a1 = (d1 >= 0) ? d1 : -d1;
    int32_t a2 = (d2 >= 0) ? d2 : -d2;
    int32_t davg_abs  = (a1 + a2) / 2;
    int32_t davg_sign = ((d1 + d2) >= 0) ? +1 : -1;

    int64_t num_abs = (int64_t)davg_abs * MM_PER_COUNT_X1000;
    int32_t dmm_abs = (int32_t)((num_abs + 500) / 1000);
    int32_t dmm_signed = APPLY_CALIB_DIST(dmm_abs) * davg_sign;

    g_dist_mm += dmm_signed;

    g_ctl_pending = 1;
    Timer_QD_ReadStatusRegister(); /* clear TC */
}

/* ===== Main control (5 ms) ===== */
static void control_step(void)
{
    const int32_t DT_MS = QD_SAMPLE_MS;

    /* 속도 */
    int32_t v_meas = mm_s_from_counts(g_c1, g_c2);
    g_v_filt = g_v_filt + (v_meas - g_v_filt) / VFILT_DEN;
    int32_t v_used = g_v_filt;

    /* 런타임 세이프 */
    if (g_run_ms < 0xFFFFFFFFu) g_run_ms += DT_MS;

    /* === START === */
    if (g_mode == MODE_START) {
        if (g_start_elapsed_ms == 0) {
            g_start_center = START_INIT_DUTY;
        } else if (g_start_center < START_MAX_DUTY) {
            int inc = (START_RAMP_PER_SEC * DT_MS) / 1000;
            if (inc < 1) inc = 1;
            g_start_center += inc;
            if (g_start_center > START_MAX_DUTY) g_start_center = START_MAX_DUTY;
        }

        /* Δcount 기반 양방향 보정 */
        int32_t r = (g_c1 >= 0) ? g_c1 : -g_c1;
        int32_t l = (g_c2 >= 0) ? g_c2 : -g_c2;
        int diff = (int)(l - r);
        int steer_enc = START_BAL_K * diff;
        if (steer_enc >  START_BAL_MAX_STEER) steer_enc =  START_BAL_MAX_STEER;
        if (steer_enc < -START_BAL_MAX_STEER) steer_enc = -START_BAL_MAX_STEER;

        int duty_right = clamp100(g_start_center + steer_enc);
        int duty_left  = clamp100(g_start_center - steer_enc);

        /* START에도 트림 적용 */
        duty_right = clamp100(apply_right_trim(duty_right));

        /* 최소 듀티 */
        if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
        if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;

        PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty_right));
        PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty_left ));

        g_start_elapsed_ms += DT_MS;

        if (g_start_elapsed_ms >= START_TOTAL_MS) {
            g_mode = MODE_RUN;
            g_integ = 0;
            int duty_ff = duty_ff_from_target_mm_s(g_v_cruise_mm_s);
            g_duty_cmd = (g_start_center > duty_ff) ? duty_ff : g_start_center;
            run_elapsed_ms = 0;
        }
        return;
    }

    /* === 세이프티/하드컷 === */
    if (g_mode == MODE_RUN) {
        if (g_dist_mm >= TARGET_DIST_MM + 100) {          /* 목표 +10cm 초과 시 강제 BRAKE */
            g_mode = MODE_BRAKE; g_brake_ms = 0; g_integ = 0; g_duty_cmd = 0;
        } else if (g_dist_mm >= MAX_TRAVEL_MM_SAFE || g_run_ms >= MAX_RUN_TIME_MS) {
            g_mode = MODE_BRAKE; g_brake_ms = 0; g_integ = 0; g_duty_cmd = 0;
        }
    }

    /* === BRAKE / DONE === */
    if (g_mode == MODE_BRAKE) {
        int32_t remain = TARGET_DIST_MM - g_dist_mm;
        static uint8_t hold_ticks = 0;

        /* 첫 틱은 강하게 쥐고 들어가도 OK */
        if (g_brake_ms == 0) {
            set_motors_symmetric(-50);
            g_brake_ms += DT_MS;
            return;
        }

        if (remain <= 0 || (v_used >= -V_STOP_THRESH_MM_S && v_used <= V_STOP_THRESH_MM_S)) {
            if (remain <= 0 && hold_ticks < HOLD_TICKS_MAX) {
                set_motors_symmetric(+HOLD_FWD_DUTY);
                hold_ticks++;
                g_brake_ms += DT_MS;
                if (g_brake_ms >= BRAKE_MAX_MS) {
                    set_motors_symmetric(0);
                    motor_enable(1u, 1u);
                    g_mode = MODE_DONE;
                }
                return;
            } else {
                hold_ticks = 0;
                set_motors_symmetric(0);
                if ( (g_dist_mm >= TARGET_DIST_MM &&
                      (v_used >= -V_STOP_THRESH_MM_S && v_used <= V_STOP_THRESH_MM_S))
                     || (g_brake_ms >= BRAKE_MAX_MS) )
                {
                    motor_enable(1u, 1u);
                    g_mode = MODE_DONE;
                }
                return;
            }
        }

        /* 역토크 감속 */
        {
            int duty = dyn_brake_duty(v_used);
            if (v_used > 0 && v_used < (2 * V_STOP_THRESH_MM_S)) {
                if (duty > (BRAKE_DUTY_MIN + 4)) duty = BRAKE_DUTY_MIN + 4;
            }
            int sgn = (v_used >= 0) ? -1 : +1;
            set_motors_symmetric(sgn * duty);
        }

        g_brake_ms += DT_MS;
        if (g_brake_ms >= BRAKE_MAX_MS) {
            set_motors_symmetric(0);
            motor_enable(1u, 1u);
            g_mode = MODE_DONE;
        }
        return;
    }
    if (g_mode == MODE_DONE) {
        set_motors_symmetric(0);
        motor_enable(1u, 1u);
        g_integ=0; g_duty_cmd=0;
        return;
    }

    /* === RUN: 센서 + (초반) 인코더 보조 === */
    light_sensors_update();

    /* 정지 트리거 (bias−, slope−) */
    {
        int32_t v_abs = (v_used >= 0) ? v_used : -v_used;
        int32_t trig_dyn = (int32_t)(STOP_TRIGGER_SLOPE_PER_V * (float)v_abs + 0.5f);  /* 음수 */

        int32_t trigger_mm =
            TARGET_DIST_MM
            - stop_dist_mm(v_used)
            + STOP_TRIGGER_BIAS_MM         /* 음수 → 더 일찍 */
            + trig_dyn;                    /* 음수 → 더 일찍 */

        if (g_dist_mm >= trigger_mm) {
            g_mode = MODE_BRAKE; g_brake_ms = 0; g_integ = 0; g_duty_cmd = 0;
            return;
        }
    }

    if (g_v_cruise_mm_s <= 0) {
        set_motors_with_trim_and_steer(0, 0);
    } else {
        int duty_ff = duty_ff_from_target_mm_s(g_v_cruise_mm_s);

        int32_t abs_c1 = (g_c1 >= 0) ? g_c1 : -g_c1;
        int32_t abs_c2 = (g_c2 >= 0) ? g_c2 : -g_c2;
        bool low_counts = (abs_c1 + abs_c2) < 2;

        int32_t SLEW_MAX = (DUTY_SLEW_PER_SEC * DT_MS) / 1000;
        if (SLEW_MAX < 1) SLEW_MAX = 1;

        if (!low_counts) {
            int32_t e = g_v_cruise_mm_s - v_used;

            int64_t i_try = (int64_t)g_integ + (int64_t)e * (int64_t)DT_MS;
            if (i_try > ICLAMP) i_try = ICLAMP;
            if (i_try < -ICLAMP) i_try = -ICLAMP;

            int32_t delta_try = ( (int64_t)KP_x1000 * e + ( (int64_t)KI_x1000 * i_try ) / 1000 ) / 1000;
            if (delta_try >  SLEW_MAX) delta_try =  SLEW_MAX;
            if (delta_try < -SLEW_MAX) delta_try = -SLEW_MAX;

            int next_center = g_duty_cmd;
            if (next_center < duty_ff - (int)SLEW_MAX)       next_center += (int)SLEW_MAX;
            else if (next_center > duty_ff + (int)SLEW_MAX)  next_center -= (int)SLEW_MAX;
            else                                             next_center  = duty_ff;

            int u_try = next_center + (int)delta_try;
            bool sat = (u_try > 100 || u_try < -100);

            if (!sat) g_integ = (int32_t)i_try;

            int32_t delta = ( (int64_t)KP_x1000 * e + ( (int64_t)KI_x1000 * g_integ ) / 1000 ) / 1000;
            if (delta >  SLEW_MAX) delta =  SLEW_MAX;
            if (delta < -SLEW_MAX) delta = -SLEW_MAX;

            int next = next_center;
            next = clamp100(next + (int)delta);
            if (next > 0 && next < DUTY_DEADBAND_PERCENT) next = DUTY_DEADBAND_PERCENT;
            g_duty_cmd = next;
        } else {
            g_integ = 0;
            g_duty_cmd = duty_ff;
        }

        /* --- 조향 --- */
        static int32_t steer_inst = 0;
        steer_inst = compute_tilt_bias_inst();

        /* RUN 초반 300ms: 센서 중립이면 인코더 미세보정 */
        if (run_elapsed_ms < RUN_ENCODER_ASSIST_MS) run_elapsed_ms += DT_MS;
        bool sensor_neutral = !(sen4_on_line || sen5_on_line || sen6_on_line);
        if (run_elapsed_ms < RUN_ENCODER_ASSIST_MS && sensor_neutral) {
            int32_t r = (g_c1 >= 0) ? g_c1 : -g_c1;
            int32_t l = (g_c2 >= 0) ? g_c2 : -g_c2;
            int diff = (int)(l - r);
            int steer_enc = diff;
            if (steer_enc >  RUN_ENCODER_ASSIST_MAX) steer_enc =  RUN_ENCODER_ASSIST_MAX;
            if (steer_enc < -RUN_ENCODER_ASSIST_MAX) steer_enc = -RUN_ENCODER_ASSIST_MAX;
            steer_inst += steer_enc;
        }

        /* 출력 */
        {
            int center = g_duty_cmd;

            int duty_right = clamp100(apply_right_trim(center + steer_inst));
            int duty_left  = clamp100(center - steer_inst);

            if (center > 0) {
                if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
                if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;
            }

            PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty_right));
            PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty_left ));
        }
    }
}

/* ===== Main ===== */
int main(void)
{
    CyGlobalIntEnable;

    /* PWM */
    Clock_PWM_Start();
    PWM_1_Start(); PWM_2_Start();
    PWM_1_WritePeriod(PWM_PERIOD);
    PWM_2_WritePeriod(PWM_PERIOD);

    motor_enable(0u, 0u);   /* enable both */

    /* Encoders + 5 ms timer */
    Clock_QENC_Start();
    QuadDec_M1_Start(); QuadDec_M2_Start();
    QuadDec_M1_SetCounter(0); QuadDec_M2_SetCounter(0);

    Clock_QD_Start();
    Timer_QD_Start();              /* TopDesign에서 5 ms로 설정 */
    isr_qd_StartEx(isr_qd_Handler);

    /* ADC for light sensors */
    ADC_Start();

    /* Init */
    g_tick = 0;
    g_c1 = g_c2 = 0;
    g_ctl_pending = 0;
    g_dist_mm = 0;
    g_v_filt = 0;
    g_integ = 0;

    g_v_cruise_mm_s = (int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100;
    g_duty_cmd = duty_ff_from_target_mm_s(g_v_cruise_mm_s);

    g_brake_ms = 0;
    g_mode = MODE_START;
    g_run_ms = 0;

    g_start_elapsed_ms = 0;
    g_start_center = START_INIT_DUTY;
    run_elapsed_ms = 0;

    set_motors_symmetric(0);

    for(;;){
        if (g_ctl_pending){
            g_ctl_pending = 0;
            control_step();  /* 5 ms */
        }
    }
}









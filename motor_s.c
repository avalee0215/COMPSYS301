// motor for the straight line
#include <project.h>
#include "motor_s.h"

// ===== User config =====
#define VMAX_CONST_MM_S        800
#define SPEED_FRAC_PERCENT     25
#define RIGHT_TRIM_PERCENT     -8

// ===== Defaults (Distance) =====
static int32_t g_target_dist_mm = 1100; //you can change the distance
#define STOP_TRIGGER_BIA_MM    (-10)

// ===== PWM / Sample =====
#define PWM_PERIOD             200u
#define QD_SAMPLE_MS           5u
#define CPR_OUTSHAFT           228u

// ===== Wheel geometry =====
#define R_MM                   34
#define PI_X1000               3142
#define PERIM_MM_X1000         ((int32_t)(2 * PI_X1000 * R_MM))
#define MM_PER_COUNT_X1000     ( PERIM_MM_X1000 / CPR_OUTSHAFT )

// ===== Calibration =====
#define CALIB_SPEED_X1000      2167
#define CALIB_DIST_X1000       1216
#define APPLY_CALIB_SPEED(x)   ( (int32_t)(((int64_t)(x) * CALIB_SPEED_X1000 + 500)/1000) )
#define APPLY_CALIB_DIST(x)    ( (int32_t)(((int64_t)(x) * CALIB_DIST_X1000  + 500)/1000) )

// ===== Polarity =====
#define RIGHT_MOTOR_SIGN       (-1)
#define LEFT_MOTOR_SIGN        (+1)
#define ENC1_SIGN              (+1)
#define ENC2_SIGN              (-1)

// ===== Safety (hard, ms-based) =====
#define MAX_RUN_TIME_MS        25000u
static uint32_t g_run_ms = 0;

// ===== Adaptive hard-stop tuning =====
#define V_STOP_THRESH_MM_S     80
#define BRAKE_OFFSET_BASE_MM   40
#define BRAKE_OFFSET_PER_MM_S  0.22f
#define BRAKE_OFFSET_MIN_MM    20
#define BRAKE_OFFSET_MAX_MM    200
#define BRAKE_DUTY_MIN         30
#define BRAKE_DUTY_PER_MM_S    0.035f
#define BRAKE_DUTY_MAX         50
#define BRAKE_MAX_MS           800u

//  Rollback guard
#define HOLD_FWD_DUTY          6
#define HOLD_TICKS_MAX         5

// ===== Control gains (5ms 기준) =====
#define KP_x1000               6
#define KI_x1000               200
#define ICLAMP                 2000000
#define DUTY_SLEW_PER_SEC      120

// FF & deadband
#define DUTY_DEADBAND_PERCENT  10
#define FF_GAIN_x1000          1000
#define FF_MAX_DUTY            95

// ===== Filtering =====
static int32_t g_v_filt = 0;
#define VFILT_DEN              4

// ===== Shared(“ISR -> control”) =====
static volatile int32_t  g_c1 = 0, g_c2 = 0;   // Δcount
static volatile int32_t  g_dist_mm = 0;
static volatile uint8_t  g_ctl_pending = 0;

// ===== Cruise & PI =====
static int               g_duty_cmd      = 50;
static int32_t           g_v_cruise_mm_s = 0;
static int32_t           g_integ         = 0;

// ===== Mode machine =====
static volatile motor_mode_t g_mode = MOTOR_START;
static uint16_t g_brake_ms = 0;

// ===== START mode: for the straight line =====
#define START_TOTAL_MS             500
#define START_INIT_DUTY            25
#define START_MAX_DUTY             38
#define START_RAMP_PER_SEC         80
#define START_BALANCE_K            4
#define START_BALANCE_MAX_OFF      2
#define START_RIGHT_KICK_MAX       0
#define START_RIGHT_KICK_THRESH    2
#define START_LEFT_DAMP_ENABLE        1
#define START_LEFT_DAMP_PCT_BASE      3
#define START_LEFT_DAMP_PCT_PER_DIFF  2
#define START_LEFT_DAMP_PCT_MAX       12
#define START_LEFT_DAMP_MS            150
static uint16_t g_start_elapsed_ms = 0;
static int      g_start_center = START_INIT_DUTY;

// ===== RUN =====
#define STALL_COUNT_THRESH        1
#define STALL_DETECT_MS           300
#define STALL_BOOST_EXTRA         6
#define BOOST_STEER_GATE          6
#define BOOST_DUTY_MIN_CENTER     12
#define BOOST_DUTY_MAX_CENTER     40
static uint16_t r_stall_acc_ms = 0;

// ===== Per-wheel minimum duty =====
#define MIN_WHEEL_DUTY_R   17
#define MIN_WHEEL_DUTY_L   13

// ===== call back =====
static motor_steer_provider_t g_steer_cb = 0;

// ===== helpers =====
static inline int clamp100(int x){ if(x>100) return 100; if(x<-100) return -100; return x; }
static inline uint16 duty_to_compare(int s){
    if (s < -100) s = -100; if (s > 100) s = 100;
    return (uint16)((PWM_PERIOD/2) + ((int32)PWM_PERIOD * s)/200);
}
static inline void set_motors_raw(int duty_right, int duty_left){
    PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty_right));
    PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty_left ));
}
static inline void set_motors_symmetric(int duty){
    duty = clamp100(duty);
    set_motors_raw(duty, duty);
}
static inline int apply_right_trim(int duty){
    int32_t scaled = ((int32_t)duty * (100 - RIGHT_TRIM_PERCENT)) / 100;
    return (int)scaled;
}
static inline void set_motors_with_trim_and_steer(int center, int steer){
    int dr = clamp100(center + steer);
    int dl = clamp100(center - steer);
    dr = clamp100(apply_right_trim(dr));
    if (center > 0){
        if (dr > 0 && dr < MIN_WHEEL_DUTY_R) dr = MIN_WHEEL_DUTY_R;
        if (dl > 0 && dl < MIN_WHEEL_DUTY_L) dl = MIN_WHEEL_DUTY_L;
    }
    set_motors_raw(dr, dl);
}
static inline int32_t mm_s_from_counts(int32_t d1, int32_t d2){
    int32_t davg = (d1 + d2)/2;
    int32_t v_mm_s_est = (int32_t)(((int64_t)davg * MM_PER_COUNT_X1000) / (int32_t)QD_SAMPLE_MS);
    return APPLY_CALIB_SPEED(v_mm_s_est);
}
static inline int duty_ff_from_target_mm_s(int32_t v_target){
    if (v_target <= 0) return 0;
    int32_t duty_lin = (int32_t)((int64_t)v_target * FF_GAIN_x1000 / VMAX_CONST_MM_S);
    int duty = (int)(duty_lin / 10);
    if (duty > 0 && duty < DUTY_DEADBAND_PERCENT) duty = DUTY_DEADBAND_PERCENT;
    if (duty > FF_MAX_DUTY) duty = FF_MAX_DUTY;
    return duty;
}
static inline int32_t dyn_brake_offset_mm(int32_t v_mm_s_filt){
    int32_t spd = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    float off = (float)BRAKE_OFFSET_BASE_MM + BRAKE_OFFSET_PER_MM_S * (float)spd;
    if (off < BRAKE_OFFSET_MIN_MM) off = BRAKE_OFFSET_MIN_MM;
    if (off > BRAKE_OFFSET_MAX_MM) off = BRAKE_OFFSET_MAX_MM;
    return (int32_t)(off + 0.5f);
}
static inline int dyn_brake_duty(int32_t v_mm_s_filt){
    int32_t spd = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    float d = (float)BRAKE_DUTY_MIN + BRAKE_DUTY_PER_MM_S * (float)spd;
    if (d > BRAKE_DUTY_MAX) d = BRAKE_DUTY_MAX;
    if (d < 0) d = 0;
    return (int)(d + 0.5f);
}
// STOP est distance
#define A_DECEL_MM_S2  3600
static inline int32_t stop_dist_mm(int32_t v_mm_s_filt){
    int64_t v = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    int64_t d = (A_DECEL_MM_S2 > 0) ? (v * v) / (2 * (int64_t)A_DECEL_MM_S2) : 0;
    if (d < 20) d = 20; if (d > 250) d = 250;
    return (int32_t)d;
}

// ===== API =====
void Motor_Init(void){
    // PWM
    Clock_PWM_Start();
    PWM_1_Start(); PWM_2_Start();
    PWM_1_WritePeriod(PWM_PERIOD);
    PWM_2_WritePeriod(PWM_PERIOD);

    // Enable motor(LOW=enable)
    CONTROL_Write(0);
    // inner status
    g_v_filt = 0; g_integ = 0; g_duty_cmd = 0;
    g_c1 = g_c2 = 0; g_dist_mm = 0; g_ctl_pending = 0;
    g_brake_ms = 0; g_mode = MOTOR_START; g_run_ms = 0;
    g_start_elapsed_ms = 0; g_start_center = START_INIT_DUTY;

    // 초기 크루즈 속도
    g_v_cruise_mm_s = (int32_t)VMAX_CONST_MM_S * (int32_t)SPEED_FRAC_PERCENT / 100;
    g_duty_cmd = duty_ff_from_target_mm_s(g_v_cruise_mm_s);

    set_motors_symmetric(0);
}
void Motor_Enable(bool enable_both){
    // bit0=M1_D1, bit1=M2_D1, HIGH=disable
    CONTROL_Write(enable_both ? 0 : 0x03);
}
void Motor_SetCruiseSpeedMmS(int32_t v_mm_s){
    if (v_mm_s < 0) v_mm_s = 0;
    if (v_mm_s > VMAX_CONST_MM_S) v_mm_s = VMAX_CONST_MM_S;
    g_v_cruise_mm_s = v_mm_s;
}
void Motor_SetTargetDistMm(int32_t target_mm){
    g_target_dist_mm = target_mm;
}
void Motor_SetSteerProvider(motor_steer_provider_t provider){
    g_steer_cb = provider;
}
void Motor_ImmediateStop(void){
    set_motors_symmetric(0);
    CONTROL_Write(0x03); // disable both
    g_mode = MOTOR_DONE;
}
void Motor_GetStatus(motor_status_t* out){
    if (!out) return;
    out->mode = g_mode;
    out->v_filt_mm_s = g_v_filt;
    out->dist_mm = g_dist_mm;
    out->duty_center = g_duty_cmd;
    // 최근 출력(대략 추정: 좌우 동일/스티어 후값 저장하려면 별도 변수 유지 가능)
    out->duty_right = 0;
    out->duty_left  = 0;
    out->run_ms = g_run_ms;
}

// ISR에서 호출: 5ms마다 Δ카운트 전달
void Motor_TickISR(int32_t raw1, int32_t raw2){
    int32_t d1 = ENC1_SIGN * raw1;
    int32_t d2 = ENC2_SIGN * raw2;
    g_c1 = d1; g_c2 = d2;

    // 거리 적산(절대 평균)
    int32_t a1 = (d1>=0)?d1:-d1, a2=(d2>=0)?d2:-d2;
    int32_t davg_abs = (a1 + a2)/2;
    int32_t dmm = (int32_t)(((int64_t)davg_abs * MM_PER_COUNT_X1000 + 500)/1000);
    dmm = APPLY_CALIB_DIST(dmm);
    g_dist_mm += dmm;

    g_ctl_pending = 1;
}

// 메인 루프의 제어 스텝(5ms)
void Motor_ControlStep(void){
    if (!g_ctl_pending) return;
    g_ctl_pending = 0;

    const int32_t DT_MS = QD_SAMPLE_MS;

    // 속도 필터
    int32_t v_meas = mm_s_from_counts(g_c1, g_c2);
    g_v_filt = g_v_filt + (v_meas - g_v_filt)/VFILT_DEN;
    int32_t v_used = g_v_filt;

    // 런타임 누적
    if (g_run_ms < 0xFFFFFFFFu) g_run_ms += DT_MS;

    // START 모드: 직선 출발(기존 로직 유지)
    if (g_mode == MOTOR_START){
        if (g_start_elapsed_ms == 0) {
            g_start_center = START_INIT_DUTY;
        } else if (g_start_center < START_MAX_DUTY) {
            int inc = (START_RAMP_PER_SEC * DT_MS)/1000;
            if (inc < 1) inc = 1;
            g_start_center += inc;
            if (g_start_center > START_MAX_DUTY) g_start_center = START_MAX_DUTY;
        }
        int32_t r = (g_c1>=0)?g_c1:-g_c1;
        int32_t l = (g_c2>=0)?g_c2:-g_c2;
        int diff = (int)(l - r);

        int right_off = 0;
        if (diff > 1){
            right_off = START_BALANCE_K * diff;
            if (right_off > START_BALANCE_MAX_OFF) right_off = START_BALANCE_MAX_OFF;
        }
        int right_kick = 0;
        if ( (l>=START_RIGHT_KICK_THRESH) && (r<=1) ){
            right_kick = START_BALANCE_K + 2;
            if (right_kick > START_RIGHT_KICK_MAX) right_kick = START_RIGHT_KICK_MAX;
        }

        int duty_right = clamp100(g_start_center + right_off + right_kick);
        int duty_left  = clamp100(g_start_center);

        if (START_LEFT_DAMP_ENABLE && g_start_elapsed_ms < START_LEFT_DAMP_MS) {
            int diff_counts = (l - r);
            if (diff_counts < 0) diff_counts = 0;
            int damp_pct = START_LEFT_DAMP_PCT_BASE + START_LEFT_DAMP_PCT_PER_DIFF * diff_counts;
            if (damp_pct > START_LEFT_DAMP_PCT_MAX) damp_pct = START_LEFT_DAMP_PCT_MAX;
            int32_t dl = duty_left;
            dl = (dl * (100 - damp_pct))/100;
            duty_left = clamp100((int)dl);
        }
        if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
        if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;

        set_motors_raw(duty_right, duty_left);

        g_start_elapsed_ms += DT_MS;
        if (g_start_elapsed_ms >= START_TOTAL_MS){
            g_mode = MOTOR_RUN;
            g_integ = 0;
            int duty_ff = duty_ff_from_target_mm_s(g_v_cruise_mm_s);
            g_duty_cmd = (g_start_center > duty_ff) ? duty_ff : g_start_center;
        }
        return;
    }

    // RUN 모드 세이프티
    if (g_mode == MOTOR_RUN){
        int32_t max_travel = g_target_dist_mm + 200; // 이전 하드 상수 유지
        if (g_dist_mm >= max_travel || g_run_ms >= MAX_RUN_TIME_MS){
            g_mode = MOTOR_BRAKE;
            g_brake_ms = 0; g_integ = 0; g_duty_cmd = 0;
        }
    }

    // BRAKE / DONE
    if (g_mode == MOTOR_BRAKE){
        int32_t remain = g_target_dist_mm - g_dist_mm;
        static uint8_t hold_ticks = 0;
        if (remain <= 0 || (v_used >= -V_STOP_THRESH_MM_S && v_used <= V_STOP_THRESH_MM_S)) {
            if (remain <= 0 && hold_ticks < HOLD_TICKS_MAX) {
                set_motors_symmetric(+HOLD_FWD_DUTY);
                hold_ticks++;
                g_brake_ms += DT_MS;
                if (g_brake_ms >= BRAKE_MAX_MS){
                    set_motors_symmetric(0);
                    CONTROL_Write(0x03); // disable
                    g_mode = MOTOR_DONE;
                }
                return;
            } else {
                hold_ticks = 0;
                set_motors_symmetric(0);
                if ( (g_dist_mm >= g_target_dist_mm &&
                      (v_used >= -V_STOP_THRESH_MM_S && v_used <= V_STOP_THRESH_MM_S))
                     || (g_brake_ms >= BRAKE_MAX_MS) )
                {
                    CONTROL_Write(0x03);
                    g_mode = MOTOR_DONE;
                }
                return;
            }
        }
        // 역토크 감속
        {
            int duty = dyn_brake_duty(v_used);
            if (v_used > 0 && v_used < (2 * V_STOP_THRESH_MM_S)) {
                if (duty > (BRAKE_DUTY_MIN + 4)) duty = BRAKE_DUTY_MIN + 4;
            }
            int sgn = (v_used >= 0) ? -1 : +1;
            set_motors_symmetric(sgn * duty);
        }
        g_brake_ms += DT_MS;
        if (g_brake_ms >= BRAKE_MAX_MS){
            set_motors_symmetric(0);
            CONTROL_Write(0x03);
            g_mode = MOTOR_DONE;
        }
        return;
    }
    if (g_mode == MOTOR_DONE){
        set_motors_symmetric(0);
        CONTROL_Write(0x03);
        g_integ=0; g_duty_cmd=0;
        return;
    }

    // === RUN: 정지 트리거 계산 ===
    {
        int32_t trigger_mm = g_target_dist_mm - stop_dist_mm(v_used) + STOP_TRIGGER_BIA_MM;
        if (g_dist_mm >= trigger_mm){
            g_mode = MOTOR_BRAKE; g_brake_ms = 0; g_integ = 0; g_duty_cmd = 0;
            return;
        }
    }

    // === RUN: FF + PI ===
    if (g_v_cruise_mm_s <= 0){
        set_motors_with_trim_and_steer(0, 0);
        return;
    }

    int duty_ff = duty_ff_from_target_mm_s(g_v_cruise_mm_s);

    int32_t abs_c1 = (g_c1>=0)?g_c1:-g_c1;
    int32_t abs_c2 = (g_c2>=0)?g_c2:-g_c2;
    bool low_counts = (abs_c1 + abs_c2) < 2;

    int32_t SLEW_MAX = (DUTY_SLEW_PER_SEC * DT_MS)/1000;
    if (SLEW_MAX < 1) SLEW_MAX = 1;

    if (!low_counts){
        int32_t e = g_v_cruise_mm_s - v_used;

        int64_t i_try = (int64_t)g_integ + (int64_t)e * (int64_t)DT_MS;
        if (i_try > ICLAMP) i_try = ICLAMP;
        if (i_try < -ICLAMP) i_try = -ICLAMP;

        int32_t delta_try = ( (int64_t)KP_x1000*e + ((int64_t)KI_x1000*i_try)/1000 )/1000;
        if (delta_try >  SLEW_MAX) delta_try =  SLEW_MAX;
        if (delta_try < -SLEW_MAX) delta_try = -SLEW_MAX;

        int next_center = g_duty_cmd;
        if      (next_center < duty_ff - (int)SLEW_MAX) next_center += (int)SLEW_MAX;
        else if (next_center > duty_ff + (int)SLEW_MAX) next_center -= (int)SLEW_MAX;
        else                                            next_center  = duty_ff;

        int u_try = next_center + (int)delta_try;
        bool sat = (u_try > 100 || u_try < -100);
        if (!sat) g_integ = (int32_t)i_try;

        int32_t delta = ( (int64_t)KP_x1000*e + ((int64_t)KI_x1000*g_integ)/1000 )/1000;
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

    // === 최종 출력: 조향(콜백) + 스톨 보정 ===
    {
        int center = g_duty_cmd;

        // 센서 쪽 즉시형 조향(NON-ACCUM) 제공
        int steer_inst = 0;
        if (g_steer_cb) steer_inst = g_steer_cb(center);

        int right_extra = 0;
        bool turning_cmd = (steer_inst != 0);  // 보수적으로 단순화
        bool small_steer = (steer_inst >= -BOOST_STEER_GATE && steer_inst <= BOOST_STEER_GATE);
        bool low_mid     = (center >= BOOST_DUTY_MIN_CENTER && center <= BOOST_DUTY_MAX_CENTER);

        if (!turning_cmd && small_steer && low_mid){
            int32_t r = (g_c1>=0)?g_c1:-g_c1, l=(g_c2>=0)?g_c2:-g_c2;
            if (r <= STALL_COUNT_THRESH && l >= 2){
                r_stall_acc_ms += DT_MS;
                if (r_stall_acc_ms >= STALL_DETECT_MS){
                    right_extra += STALL_BOOST_EXTRA;
                    r_stall_acc_ms = 0;
                }
            } else r_stall_acc_ms = 0;
        } else r_stall_acc_ms = 0;

        int duty_right = clamp100(apply_right_trim(center + steer_inst + right_extra));
        int duty_left  = clamp100(center - steer_inst);

        if (center > 0){
            if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
            if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;
        }
        set_motors_raw(duty_right, duty_left);
    }
}



#include "motor_s.h"

/* RIGHT_TRIM_PERCENT가 다른 헤더/파일에 있으면 그 값을 사용.
 * 없으면 0(트림 없음)으로 처리.
 */
#ifndef RIGHT_TRIM_PERCENT
#define RIGHT_TRIM_PERCENT   0
#endif

int clamp100(int x){
    if (x > 100) return 100;
    if (x < -100) return -100;
    return x;
}

uint16 duty_to_compare(int s){
    if (s < -100) s = -100;
    if (s >  100) s =  100;
    /* PWM Period의 중앙을 0%로 보고, ±로 오프셋 */
    return (uint16)((PWM_PERIOD/2) + ((int32)PWM_PERIOD * s)/200);
}

void motor_enable(uint8 m1_disable, uint8 m2_disable){
    uint8 v = 0;
    if (m1_disable) v |= 0x01;  /* bit0 -> M1_D1 (HIGH=disable) */
    if (m2_disable) v |= 0x02;  /* bit1 -> M2_D1 (HIGH=disable) */
    CONTROL_Write(v);
}

int apply_right_trim(int duty){
    /* RIGHT_TRIM_PERCENT: 음수면 오른쪽 강화(네 기존 로직 유지) */
    int32_t scaled = ((int32_t)duty * (100 - RIGHT_TRIM_PERCENT)) / 100;
    return (int)scaled;
}

void set_motors_symmetric(int duty){
    duty = clamp100(duty);
    PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty));
    PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty));
}

void set_motors_with_trim_and_steer(int duty_center, int steer){
    int duty_right = clamp100(duty_center + steer);
    int duty_left  = clamp100(duty_center - steer);

    /* 오른쪽만 트림 */
    duty_right = clamp100(apply_right_trim(duty_right));

    /* 전진 구간에서 최소 듀티 보장 */
    if (duty_center > 0) {
        if (duty_right > 0 && duty_right < MIN_WHEEL_DUTY_R) duty_right = MIN_WHEEL_DUTY_R;
        if (duty_left  > 0 && duty_left  < MIN_WHEEL_DUTY_L) duty_left  = MIN_WHEEL_DUTY_L;
    }

    PWM_1_WriteCompare(duty_to_compare(RIGHT_MOTOR_SIGN * duty_right));
    PWM_2_WriteCompare(duty_to_compare(LEFT_MOTOR_SIGN  * duty_left ));
}

int dyn_brake_duty(int32_t v_mm_s_filt){
    /* 속도 크기에 따라 브레이크 듀티 가변 (네 v4 공식 그대로) */
    int32_t spd = (v_mm_s_filt >= 0) ? v_mm_s_filt : -v_mm_s_filt;
    float d = (float)BRAKE_DUTY_MIN + BRAKE_DUTY_PER_MM_S * (float)spd;
    if (d > BRAKE_DUTY_MAX) d = BRAKE_DUTY_MAX;
    if (d < 0) d = 0;
    return (int)(d + 0.5f);
}


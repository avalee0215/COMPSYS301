#pragma once
#include <stdint.h>
#include <project.h>

/*
 * NOTE:
 * 아래 매크로들은 보통 main.c(혹은 config 헤더)에 이미 정의되어 있습니다.
 * 만약 미정의 상태면 여기의 기본값이 사용됩니다.
 */
#ifndef PWM_PERIOD
#define PWM_PERIOD             200u
#endif

#ifndef RIGHT_MOTOR_SIGN
#define RIGHT_MOTOR_SIGN       (-1)   /* M1 = Right */
#endif
#ifndef LEFT_MOTOR_SIGN
#define LEFT_MOTOR_SIGN        (+1)   /* M2 = Left  */
#endif

#ifndef MIN_WHEEL_DUTY_R
#define MIN_WHEEL_DUTY_R       15
#endif
#ifndef MIN_WHEEL_DUTY_L
#define MIN_WHEEL_DUTY_L       15
#endif

#ifndef BRAKE_DUTY_MIN
#define BRAKE_DUTY_MIN         30
#endif
#ifndef BRAKE_DUTY_MAX
#define BRAKE_DUTY_MAX         55
#endif
#ifndef BRAKE_DUTY_PER_MM_S
#define BRAKE_DUTY_PER_MM_S    0.035f
#endif

/* ===== Public API ===== */

/* 범위 [-100..100]로 클램프 */
int  clamp100(int x);

/* 듀티[%](-100..100) -> PWM 비교값 */
uint16 duty_to_compare(int duty_percent);

/* 모터 드라이버 enable/disable (HIGH=disable) */
void motor_enable(uint8 m1_disable, uint8 m2_disable);

/* 오른쪽 트림 적용 (RIGHT_TRIM_PERCENT가 main쪽에 있으면 그 값을 사용함) */
int  apply_right_trim(int duty);

/* 양쪽 같은 듀티 출력 (부호/모터극성 반영) */
void set_motors_symmetric(int duty_center);

/* 센터+조향(±steer)로 좌/우 계산 + 트림/최소듀티 적용 후 출력 */
void set_motors_with_trim_and_steer(int duty_center, int steer);

/* 속도 기반 동적 브레이크 듀티 계산 (입력: v_mm_s) */
int  dyn_brake_duty(int32_t v_mm_s_filt);


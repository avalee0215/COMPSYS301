// motor for the straight line
#pragma once
#include <stdint.h>
#include <stdbool.h>

// RUN MODE
typedef enum { MOTOR_START=0, MOTOR_RUN, MOTOR_BRAKE, MOTOR_DONE } motor_mode_t;

// MOTOR STATUS
typedef struct {
    motor_mode_t mode;
    int32_t      v_filt_mm_s;
    int32_t      dist_mm;
    int          duty_center;   // center duty (조향 전)
    int          duty_right;    
    int          duty_left;     
    uint32_t     run_ms;
} motor_status_t;


//  - 인자: 현재 center duty
//  - 반환: steer(왼쪽+, 오른쪽-), 범위는 [-100..100] 내로 가정
typedef int (*motor_steer_provider_t)(int duty_center);

// 초기화/설정
void Motor_Init(void);
void Motor_Enable(bool enable_both); // true: enable, false: disable
void Motor_SetCruiseSpeedMmS(int32_t v_mm_s);
void Motor_SetTargetDistMm(int32_t target_mm);
void Motor_SetSteerProvider(motor_steer_provider_t provider);

// 5ms 타이머 ISR에서 호출 (엔코더 증분 전달)
void Motor_TickISR(int32_t enc1_counts, int32_t enc2_counts);

// 메인 루프에서 주기적으로 호출(5ms마다 한 번)
void Motor_ControlStep(void);

// 강제 정지(긴급)
void Motor_ImmediateStop(void);

// 상태 읽기(디버그/테스트용)
void Motor_GetStatus(motor_status_t* out);


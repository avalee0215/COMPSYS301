#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include <sensors.h>
#include "motor_s.h"   

/* ===== Light sensors  ===== */
static uint8_t sen1_on_line=0, sen2_on_line=0, sen3_on_line=0;
static uint8_t sen4_on_line=0, sen5_on_line=0, sen6_on_line=0;
static uint8_t g_direction = 0; /* 1=left, 2=right */

// scaled_steer_step() 
static inline int scaled_steer_step(int base_duty) {
    const int STEER_STEP = 1, MIN_STEER_STEP = 1;
    int pct = (base_duty >= 0) ? base_duty : -base_duty;
    int step = (pct < 50) ? (STEER_STEP * (pct + 10) + 59) / 60 : STEER_STEP;
    if (step < MIN_STEER_STEP) step = MIN_STEER_STEP;
    return step;
}

static void light_sensors_update(void){
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

/* ===  ===
 *  motor module call back
 */
static int steer_bias_provider(int center_duty){
    int small = scaled_steer_step(center_duty);
    int big   = (small*3+1)/2;

    if (sen6_on_line && sen4_on_line && !sen5_on_line) return +small;
    if (sen6_on_line && sen5_on_line && !sen4_on_line) return -small;
    if (!sen6_on_line && sen4_on_line && !sen5_on_line) return +big;
    if (!sen6_on_line && sen5_on_line && !sen4_on_line) return -big;
    return 0;
}

/* ===== 5 ms Timer ISR ===== */
CY_ISR(isr_qd_Handler)
{
    int32_t raw1 = QuadDec_M1_GetCounter();  QuadDec_M1_SetCounter(0);
    int32_t raw2 = QuadDec_M2_GetCounter();  QuadDec_M2_SetCounter(0);

    Motor_TickISR(raw1, raw2);               
    Timer_QD_ReadStatusRegister();           // clear TC
}

int main(void)
{
    CyGlobalIntEnable;

    // Encoders + 5 ms timer
    Clock_QENC_Start();
    QuadDec_M1_Start(); QuadDec_M2_Start();
    QuadDec_M1_SetCounter(0); QuadDec_M2_SetCounter(0);

    Clock_QD_Start();
    Timer_QD_Start();              // TopDesign 5 ms
    isr_qd_StartEx(isr_qd_Handler);

    // ADC for light sensors
    ADC_Start();


    Motor_Init();
    Motor_SetTargetDistMm(1100);
    Motor_SetCruiseSpeedMmS((int32_t)800 * 25 / 100);
    Motor_SetSteerProvider(steer_bias_provider);

    for(;;){
        light_sensors_update();     // 센서 갱신(빠르게 돌려도 OK)
        Motor_ControlStep();        // 5 ms마다 1회 내부 플래그로 실행
    }
}

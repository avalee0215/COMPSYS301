#include <project.h>
#include <stdint.h>
#include "directions.h"
#include "motor_s.h"

/* ================================
 * Robot-specific PWM + enable
 *  - Clock_PWM, PWM_1, PWM_2
 *  - CONTROL pin: LOW = enable
 *  - Sign map (typical from your build):
 *      LEFT  forward  = +duty
 *      RIGHT forward  = -duty
 *    -> Pivot LEFT  = left reverse,  right forward
 *    -> Pivot RIGHT = right reverse, left  forward
 * Adjust RIGHT/LEFT signs below if your wiring differs.
 * ================================ */

/* ===== Tunables (start here, then fine-tune on floor) ===== */
#define TURN_SPEED_PC        28      // 10..40%
#define TURN_TIME_MS         550     // ~90° at TURN_SPEED_PC (tune ±20–60 ms)
#define STOP_BEFORE_MS       80      // settle before pivot
#define BRAKE_AFTER_MS       90      // quick brake after pivot
#define PWM_PERIOD_TICKS     200u    // must match your PWM config

/* NEW: small holdoff before we start the pivot.
 * Lets straight-line control run ~10 ms longer after detection. */
#define TURN_START_DELAY_MS  70

/* Polarity (forward direction) */
#define RIGHT_MOTOR_SIGN     (-1)
#define LEFT_MOTOR_SIGN      (+1)

static inline void pivot_steer(int steer_pc) {
    set_motors_with_trim_and_steer(0, steer_pc);
}

static void brake_quick(uint16_t ms, int base_pc){
    int d = (base_pc < 8) ? 8 : base_pc;
    set_motors_symmetric(-d);   
    CyDelay(ms);
}

void Directions_Init(void){
    // nothing needed; safe to call anytime
}

bool Directions_IsBusy(void){
    return false; // blocking implementation
}


void Directions_Handle(volatile uint8_t* gdir){
    if (!gdir || *gdir==0u)return;
    
    CyDelay(TURN_START_DELAY_MS);
    
    set_motors_symmetric(0);
    CyDelay(STOP_BEFORE_MS);
    
    if (*gdir == 1u) {                 // LEFT
        pivot_steer(+TURN_SPEED_PC);
        CyDelay(TURN_TIME_MS);
    } else {                            // RIGHT
        pivot_steer(-TURN_SPEED_PC);
        CyDelay(TURN_TIME_MS);
    }
    brake_quick(BRAKE_AFTER_MS, TURN_SPEED_PC);
    set_motors_symmetric(0);
    
        *gdir = 0u;            // back to straight mode
    
}

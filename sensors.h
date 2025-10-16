#ifndef SENSOR_H 
#define SENSOR_H

#include <stdint.h> 
#include <stdlib.h>
#include <stdio.h>

// Number of ADC samples per window (the buffer size)
// Larger N_SAMPLES = smoother peak to peak measurement, but response slows down 
#define N_SAMPLES 1700 

// ADC reference voltage (since using Vssa to Vdda, this is boards supply of 5V)
#define REF_MV 5000 

    
// Function prototype for peak-peak calculation 
uint16_t Sensor_ComputePeakToPeak(uint8_t channel);

#endif
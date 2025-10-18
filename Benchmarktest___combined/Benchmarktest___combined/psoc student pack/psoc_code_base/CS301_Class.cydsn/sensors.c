#include <project.h> 
#include <sensors.h>
#include <stdint.h> 
#include <stdlib.h>
#include <stdio.h>

// Computes the peak-peak value (max - min) of N_SAMPLES from ADC 
// for the selected channel (eg channel 0 = A0) 
uint16_t Sensor_ComputePeakToPeak(uint8_t channel) {
    
uint16_t min_val = 4095; // start with max possible 
uint16_t max_val = 0; // start with min possible 
    
  ADC_StartConvert();

  // collect N_SAMPLES from ADC 
  for (int i = 0; i < N_SAMPLES; i++){
    // Read the result from the given ADC channel (0-7 for A0-A7) 
    uint16_t sample = ADC_GetResult16(channel); 
    // wait until ADC conversion for current channel is done 

    // Update max and min trackers
    if(sample < min_val) {
        min_val = sample;
    }
    if(sample > max_val) {
        max_val = sample; 
    }
} 
  ADC_StopConvert();

  // return peak to peak voltage in ADC counts 
 return (max_val - min_val);

}
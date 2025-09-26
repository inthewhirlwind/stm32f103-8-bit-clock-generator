#ifndef CLOCK_GENERATOR_H
#define CLOCK_GENERATOR_H

#include <stdint.h>
#include "config.h"

// Potentiometer input mapping functions
uint32_t map_potentiometer_to_frequency(uint16_t adc_value);
uint32_t calculate_timer_period(uint32_t frequency_hz);

// Hardware initialization functions
void init_adc(void);
void init_timer_pwm(void);
void init_clock_generator(void);

// Main clock generator control functions
uint16_t read_potentiometer(void);
void set_output_frequency(uint32_t frequency_hz);
void update_clock_output(void);

#endif // CLOCK_GENERATOR_H
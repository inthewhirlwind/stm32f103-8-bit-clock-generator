/**
 * Host-side test runner for frequency mapping validation
 * Compile with: gcc -DUNIT_TEST_MODE -I inc src/test_frequency_mapping.c test_runner.c -o test_runner
 */

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

// Mock the config.h constants for host testing
#define MIN_FREQUENCY_HZ        1
#define LOW_RANGE_MAX_HZ        100
#define MAX_FREQUENCY_HZ        100000
#define ADC_MAX_VALUE           4095
#define LOW_RANGE_PERCENTAGE    20
#define LOW_RANGE_THRESHOLD     ((ADC_MAX_VALUE * LOW_RANGE_PERCENTAGE) / 100)
#define TIMER_CLOCK_HZ          1000000

// Include the mapping function implementations
uint32_t map_potentiometer_to_frequency(uint16_t adc_value)
{
    uint32_t frequency;
    
    if (adc_value <= LOW_RANGE_THRESHOLD) {
        // First 20%: Linear mapping from 1Hz to 100Hz
        frequency = MIN_FREQUENCY_HZ + ((uint32_t)adc_value * (LOW_RANGE_MAX_HZ - MIN_FREQUENCY_HZ)) / LOW_RANGE_THRESHOLD;
    } else {
        // Remaining 80%: Linear mapping from 100Hz to 100kHz
        uint32_t adjusted_adc = adc_value - LOW_RANGE_THRESHOLD - 1;
        uint32_t high_range = ADC_MAX_VALUE - LOW_RANGE_THRESHOLD;
        
        // Special case for maximum ADC value to ensure we hit exactly MAX_FREQUENCY_HZ
        if (adc_value == ADC_MAX_VALUE) {
            frequency = MAX_FREQUENCY_HZ;
        } else {
            frequency = LOW_RANGE_MAX_HZ + (adjusted_adc * (MAX_FREQUENCY_HZ - LOW_RANGE_MAX_HZ)) / high_range;
        }
    }
    
    return frequency;
}

uint32_t calculate_timer_period(uint32_t frequency_hz)
{
    if (frequency_hz == 0) {
        return 0;
    }
    return (TIMER_CLOCK_HZ / frequency_hz) - 1;
}

// Include test functions
#define UNIT_TEST_MODE
#include "src/test_frequency_mapping.c"
#include "clock_generator.h"
#include <stdio.h>
#include <assert.h>

/**
 * Test the frequency mapping function to ensure it meets specifications
 */
void test_frequency_mapping(void)
{
    // Test cases for the first 20% (0-819): 1Hz to 100Hz
    printf("Testing first 20%% range (1Hz - 100Hz):\n");
    
    uint32_t freq_0 = map_potentiometer_to_frequency(0);
    printf("ADC 0: %lu Hz (expected: 1 Hz)\n", freq_0);
    assert(freq_0 == 1);
    
    uint32_t freq_410 = map_potentiometer_to_frequency(410); // ~50% of first range
    printf("ADC 410: %lu Hz (expected: ~50 Hz)\n", freq_410);
    assert(freq_410 >= 49 && freq_410 <= 51); // Allow small rounding error
    
    uint32_t freq_819 = map_potentiometer_to_frequency(819);
    printf("ADC 819: %lu Hz (expected: 100 Hz)\n", freq_819);
    assert(freq_819 == 100);
    
    // Test cases for the remaining 80% (820-4095): 100Hz to 100kHz
    printf("\nTesting remaining 80%% range (100Hz - 100kHz):\n");
    
    uint32_t freq_820 = map_potentiometer_to_frequency(820);
    printf("ADC 820: %lu Hz (expected: 100 Hz)\n", freq_820);
    assert(freq_820 == 100);
    
    uint32_t freq_2457 = map_potentiometer_to_frequency(2457); // ~50% of second range
    printf("ADC 2457: %lu Hz (expected: ~50000 Hz)\n", freq_2457);
    assert(freq_2457 >= 49000 && freq_2457 <= 51000); // Allow rounding error
    
    uint32_t freq_4095 = map_potentiometer_to_frequency(4095);
    printf("ADC 4095: %lu Hz (expected: 100000 Hz)\n", freq_4095);
    assert(freq_4095 == 100000);
    
    // Additional boundary tests
    printf("\nAdditional boundary tests:\n");
    
    // Test just before threshold
    uint32_t freq_818 = map_potentiometer_to_frequency(818);
    printf("ADC 818: %lu Hz (should be < 100 Hz)\n", freq_818);
    assert(freq_818 < 100);
    
    // Test just after threshold
    uint32_t freq_821 = map_potentiometer_to_frequency(821);
    printf("ADC 821: %lu Hz (should be > 100 Hz)\n", freq_821);
    assert(freq_821 > 100);
    
    printf("\nAll frequency mapping tests passed!\n");
}

/**
 * Test timer period calculations
 */
void test_timer_calculations(void)
{
    printf("\nTesting timer period calculations:\n");
    
    uint32_t period_1hz = calculate_timer_period(1);
    printf("1 Hz: period = %lu (expected: 999999)\n", period_1hz);
    assert(period_1hz == 999999);
    
    uint32_t period_100hz = calculate_timer_period(100);
    printf("100 Hz: period = %lu (expected: 9999)\n", period_100hz);
    assert(period_100hz == 9999);
    
    uint32_t period_1khz = calculate_timer_period(1000);
    printf("1 kHz: period = %lu (expected: 999)\n", period_1khz);
    assert(period_1khz == 999);
    
    uint32_t period_100khz = calculate_timer_period(100000);
    printf("100 kHz: period = %lu (expected: 9)\n", period_100khz);
    assert(period_100khz == 9);
    
    printf("All timer calculation tests passed!\n");
}

/**
 * Generate comprehensive frequency mapping table
 */
void print_frequency_table(void)
{
    printf("\n\nComprehensive Frequency Mapping Table:\n");
    printf("=====================================\n");
    printf("ADC Value | Frequency (Hz) | Range\n");
    printf("----------|----------------|--------\n");
    
    // Test key points in first 20%
    for (int adc = 0; adc <= 819; adc += 163) { // Every ~20% of first range
        uint32_t freq = map_potentiometer_to_frequency(adc);
        printf("%9d | %13lu | Low\n", adc, freq);
    }
    
    // Ensure we test exactly at threshold
    uint32_t freq_819 = map_potentiometer_to_frequency(819);
    printf("%9d | %13lu | Low\n", 819, freq_819);
    
    // Test key points in remaining 80%
    for (int adc = 820; adc <= 4095; adc += 655) { // Every ~20% of second range
        uint32_t freq = map_potentiometer_to_frequency(adc);
        printf("%9d | %13lu | High\n", adc, freq);
    }
    
    // Ensure we test exactly at maximum
    uint32_t freq_4095 = map_potentiometer_to_frequency(4095);
    printf("%9d | %13lu | High\n", 4095, freq_4095);
}

#ifdef UNIT_TEST_MODE
int main(void)
{
    printf("STM32F103 Clock Generator - Frequency Mapping Tests\n");
    printf("==================================================\n");
    
    test_frequency_mapping();
    test_timer_calculations();
    print_frequency_table();
    
    printf("\nAll tests completed successfully!\n");
    return 0;
}
#endif
#ifndef CONFIG_H
#define CONFIG_H

/* Hardware Configuration */
#define SYSTEM_CLOCK_MHZ        72      // System clock frequency in MHz
#define HSE_FREQUENCY_MHZ       8       // External crystal frequency

/* GPIO Pin Configuration */
#define POTENTIOMETER_ADC_PIN   GPIO_PIN_0      // PA0 - ADC input
#define POTENTIOMETER_ADC_PORT  GPIOA
#define POTENTIOMETER_ADC_CHANNEL ADC_CHANNEL_0

#define CLOCK_OUTPUT_PIN        GPIO_PIN_1      // PA1 - PWM output  
#define CLOCK_OUTPUT_PORT       GPIOA
#define CLOCK_OUTPUT_CHANNEL    TIM_CHANNEL_2

/* Frequency Range Configuration */
#define MIN_FREQUENCY_HZ        1       // Updated from 10Hz to 1Hz per requirements
#define LOW_RANGE_MAX_HZ        100     // Transition point between ranges
#define MAX_FREQUENCY_HZ        100000  // Maximum output frequency (100kHz)

/* Potentiometer Mapping Configuration */ 
#define ADC_RESOLUTION_BITS     12      // STM32F103 has 12-bit ADC
#define ADC_MAX_VALUE           ((1 << ADC_RESOLUTION_BITS) - 1)  // 4095
#define LOW_RANGE_PERCENTAGE    20      // First 20% for low frequency range
#define LOW_RANGE_THRESHOLD     ((ADC_MAX_VALUE * LOW_RANGE_PERCENTAGE) / 100)  // 819

/* Timer Configuration */
#define TIMER_PRESCALER         (SYSTEM_CLOCK_MHZ - 1)  // Gives 1MHz timer clock
#define TIMER_CLOCK_HZ          (SYSTEM_CLOCK_MHZ * 1000000 / (TIMER_PRESCALER + 1))

/* Application Configuration */
#define ADC_UPDATE_INTERVAL_MS  10      // How often to read potentiometer
#define PWM_DUTY_CYCLE_PERCENT  50      // 50% duty cycle for clock output

/* Debug Configuration */
#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

/* Validation Macros */
#if LOW_RANGE_THRESHOLD >= ADC_MAX_VALUE
    #error "Low range threshold must be less than ADC maximum value"
#endif

#if MIN_FREQUENCY_HZ >= LOW_RANGE_MAX_HZ
    #error "Minimum frequency must be less than low range maximum"
#endif

#if LOW_RANGE_MAX_HZ >= MAX_FREQUENCY_HZ
    #error "Low range maximum must be less than overall maximum frequency"
#endif

#endif // CONFIG_H
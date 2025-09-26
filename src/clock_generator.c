#include "clock_generator.h"
#include "config.h"
#include "stm32f1xx_hal.h"

// Global variables
static ADC_HandleTypeDef hadc1;
static TIM_HandleTypeDef htim2;

/**
 * Maps potentiometer ADC value to frequency according to specification:
 * - First 20% (0-819): Linear mapping from 1Hz to 100Hz
 * - Remaining 80% (820-4095): Linear mapping from 100Hz to 100kHz
 * 
 * @param adc_value: 12-bit ADC value (0-4095)
 * @return: Frequency in Hz
 */
uint32_t map_potentiometer_to_frequency(uint16_t adc_value)
{
    uint32_t frequency;
    
    if (adc_value <= LOW_RANGE_THRESHOLD) {
        // First 20%: Linear mapping from 1Hz to 100Hz
        // frequency = 1 + (adc_value * (100 - 1)) / LOW_RANGE_THRESHOLD
        frequency = MIN_FREQUENCY_HZ + ((uint32_t)adc_value * (LOW_RANGE_MAX_HZ - MIN_FREQUENCY_HZ)) / LOW_RANGE_THRESHOLD;
    } else {
        // Remaining 80%: Linear mapping from 100Hz to 100kHz
        // Map (820-4095) to (100Hz-100kHz)
        uint32_t adjusted_adc = adc_value - LOW_RANGE_THRESHOLD - 1; // 0-3275 range
        uint32_t high_range = ADC_MAX_VALUE - LOW_RANGE_THRESHOLD; // 3276
        
        // Special case for maximum ADC value to ensure we hit exactly MAX_FREQUENCY_HZ
        if (adc_value == ADC_MAX_VALUE) {
            frequency = MAX_FREQUENCY_HZ;
        } else {
            frequency = LOW_RANGE_MAX_HZ + (adjusted_adc * (MAX_FREQUENCY_HZ - LOW_RANGE_MAX_HZ)) / high_range;
        }
    }
    
    return frequency;
}

/**
 * Calculate timer period for given frequency
 * Assuming 72MHz system clock with prescaler
 * 
 * @param frequency_hz: Target frequency in Hz
 * @return: Timer period value
 */
uint32_t calculate_timer_period(uint32_t frequency_hz)
{
    // System clock = 72MHz, using prescaler of 72 gives 1MHz timer clock
    // Period = TIMER_CLOCK_HZ / frequency_hz
    
    if (frequency_hz == 0) {
        return 0;
    }
    
    return (TIMER_CLOCK_HZ / frequency_hz) - 1;
}

/**
 * Initialize ADC1 for potentiometer reading
 */
void init_adc(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Enable ADC1 clock
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure GPIO pin for ADC input
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = POTENTIOMETER_ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(POTENTIOMETER_ADC_PORT, &GPIO_InitStruct);
    
    // Configure ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);
    
    // Configure ADC channel
    sConfig.Channel = POTENTIOMETER_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    // Calibrate ADC
    HAL_ADCEx_Calibration_Start(&hadc1);
}

/**
 * Initialize Timer 2 for PWM output
 */
void init_timer_pwm(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    // Enable TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure output pin for TIM2 PWM
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = CLOCK_OUTPUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CLOCK_OUTPUT_PORT, &GPIO_InitStruct);
    
    // Configure timer
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIMER_PRESCALER; // 72MHz / 72 = 1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999; // Will be updated dynamically
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);
    
    // Configure PWM channel
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500; // Will be set to 50% duty cycle dynamically
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, CLOCK_OUTPUT_CHANNEL);
}

/**
 * Initialize the complete clock generator system
 */
void init_clock_generator(void)
{
    // Initialize HAL
    HAL_Init();
    
    // Configure system clock to 72MHz
    SystemClock_Config();
    
    // Initialize peripherals
    init_adc();
    init_timer_pwm();
    
    // Start PWM output
    HAL_TIM_PWM_Start(&htim2, CLOCK_OUTPUT_CHANNEL);
}

/**
 * Read potentiometer value via ADC
 * 
 * @return: 12-bit ADC value (0-4095)
 */
uint16_t read_potentiometer(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}

/**
 * Set output frequency by updating timer period
 * 
 * @param frequency_hz: Target frequency in Hz
 */
void set_output_frequency(uint32_t frequency_hz)
{
    uint32_t period = calculate_timer_period(frequency_hz);
    
    if (period > 0) {
        __HAL_TIM_SET_AUTORELOAD(&htim2, period);
        __HAL_TIM_SET_COMPARE(&htim2, CLOCK_OUTPUT_CHANNEL, (period * PWM_DUTY_CYCLE_PERCENT) / 100);
    }
}

/**
 * Update clock output based on current potentiometer position
 */
void update_clock_output(void)
{
    uint16_t adc_value = read_potentiometer();
    uint32_t frequency = map_potentiometer_to_frequency(adc_value);
    set_output_frequency(frequency);
}

// Weak definition of SystemClock_Config - should be implemented in main.c
__weak void SystemClock_Config(void)
{
    // Default implementation - configure 72MHz system clock
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
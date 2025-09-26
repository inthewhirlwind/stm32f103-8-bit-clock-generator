#include "clock_generator.h"

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    /* Enable HSE oscillator */
    LL_RCC_HSE_Enable();
    while(LL_RCC_HSE_IsReady() != 1);

    /* Configure PLL */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1);

    /* Configure system clock */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    /* Update SystemCoreClock variable */
    SystemCoreClock = 72000000;
}

/**
 * @brief GPIO Initialization for Timer outputs
 */
void GPIO_Init(void)
{
    /* Enable GPIO clocks */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

    /* Configure TIM1 CH1 (PA8) - Normal polarity */
    LL_GPIO_SetPinMode(TIM1_CH1_GPIO_PORT, TIM1_CH1_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(TIM1_CH1_GPIO_PORT, TIM1_CH1_GPIO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(TIM1_CH1_GPIO_PORT, TIM1_CH1_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    /* Configure TIM1 CH2 (PA9) - Inverse polarity */
    LL_GPIO_SetPinMode(TIM1_CH2_GPIO_PORT, TIM1_CH2_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(TIM1_CH2_GPIO_PORT, TIM1_CH2_GPIO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(TIM1_CH2_GPIO_PORT, TIM1_CH2_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    /* Configure TIM2 CH3 (PA2) - Normal polarity */
    LL_GPIO_SetPinMode(TIM2_CH3_GPIO_PORT, TIM2_CH3_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(TIM2_CH3_GPIO_PORT, TIM2_CH3_GPIO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(TIM2_CH3_GPIO_PORT, TIM2_CH3_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    /* Configure TIM2 CH4 (PA3) - Inverse polarity */
    LL_GPIO_SetPinMode(TIM2_CH4_GPIO_PORT, TIM2_CH4_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(TIM2_CH4_GPIO_PORT, TIM2_CH4_GPIO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(TIM2_CH4_GPIO_PORT, TIM2_CH4_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
}

/**
 * @brief Timer 1 Initialization
 * Channel A (CH1) - Normal polarity
 * Channel B (CH2) - Inverse polarity relative to CH1
 */
void Timer1_Init(void)
{
    /* Enable Timer 1 clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Configure Timer 1 base */
    LL_TIM_SetPrescaler(TIM1, TIM1_PRESCALER);
    LL_TIM_SetAutoReload(TIM1, 100);  // 10kHz output frequency
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);

    /* Configure Channel 1 (Normal polarity) */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetCompareCH1(TIM1, 50);  // 50% duty cycle

    /* Configure Channel 2 (Inverse polarity relative to CH1) */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_LOW);  // Inverse polarity
    LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetCompareCH2(TIM1, 50);  // Same compare value as CH1

    /* Enable outputs */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    /* Enable main output (required for TIM1) */
    LL_TIM_EnableAllOutputs(TIM1);
}

/**
 * @brief Timer 2 Initialization  
 * Channel C (CH3) - Normal polarity
 * Channel D (CH4) - Inverse polarity relative to CH3
 */
void Timer2_Init(void)
{
    /* Enable Timer 2 clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* Configure Timer 2 base */
    LL_TIM_SetPrescaler(TIM2, TIM2_PRESCALER);
    LL_TIM_SetAutoReload(TIM2, 200);  // 5kHz output frequency
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);

    /* Configure Channel 3 (Normal polarity) */
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetCompareCH3(TIM2, 100);  // 50% duty cycle

    /* Configure Channel 4 (Inverse polarity relative to CH3) */
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);  // Inverse polarity
    LL_TIM_OC_SetCompareCH4(TIM2, 100);  // Same compare value as CH3

    /* Enable outputs */
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
}

/**
 * @brief Initialize the complete clock generator system
 */
void ClockGenerator_Init(void)
{
    SystemClock_Config();
    GPIO_Init();
    Timer1_Init();
    Timer2_Init();
}

/**
 * @brief Start the clock generator timers
 */
void ClockGenerator_Start(void)
{
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM2);
}

/**
 * @brief Stop the clock generator timers
 */
void ClockGenerator_Stop(void)
{
    LL_TIM_DisableCounter(TIM1);
    LL_TIM_DisableCounter(TIM2);
}

/**
 * @brief Set the output frequency for both timers
 * @param frequency: Desired frequency in Hz
 */
void ClockGenerator_SetFrequency(uint32_t frequency)
{
    uint32_t auto_reload = (TIM1_FREQUENCY / frequency) - 1;
    
    /* Update Timer 1 */
    LL_TIM_SetAutoReload(TIM1, auto_reload);
    LL_TIM_OC_SetCompareCH1(TIM1, auto_reload / 2);
    LL_TIM_OC_SetCompareCH2(TIM1, auto_reload / 2);
    
    /* Update Timer 2 */
    LL_TIM_SetAutoReload(TIM2, auto_reload);
    LL_TIM_OC_SetCompareCH3(TIM2, auto_reload / 2);
    LL_TIM_OC_SetCompareCH4(TIM2, auto_reload / 2);
}
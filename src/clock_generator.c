#include "clock_generator.h"

/* System state variable */
static system_state_t g_system_state = {0};

/* System tick counter for timing */
static volatile uint32_t g_system_tick = 0;

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
    
    /* Initialize SysTick for 1ms ticks */
    LL_Init1msTick(SystemCoreClock);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();
}

/**
 * @brief GPIO Initialization for all peripherals
 */
void GPIO_Init(void)
{
    /* Enable GPIO clocks */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

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

    /* Configure Potentiometer (PA0) - Analog input */
    LL_GPIO_SetPinMode(POTENTIOMETER_PORT, POTENTIOMETER_PIN, LL_GPIO_MODE_ANALOG);
}

/**
 * @brief Timer 1 Initialization with inverse polarity
 * Channel A (CH1) - Normal polarity
 * Channel B (CH2) - Inverse polarity relative to CH1
 */
void Timer1_Init(void)
{
    /* Enable Timer 1 clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Configure Timer 1 base */
    LL_TIM_SetPrescaler(TIM1, TIM1_PRESCALER);
    LL_TIM_SetAutoReload(TIM1, 1000);  // Default to 1kHz
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);

    /* Configure Channel 1 (Normal polarity) */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
    LL_TIM_OC_SetCompareCH1(TIM1, 500);  // 50% duty cycle

    /* Configure Channel 2 (Inverse polarity relative to CH1) */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_LOW);  // Inverse polarity
    LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_HIGH);
    LL_TIM_OC_SetCompareCH2(TIM1, 500);  // Same compare value as CH1

    /* Enable outputs */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    /* Enable main output (required for TIM1) */
    LL_TIM_EnableAllOutputs(TIM1);
}

/**
 * @brief Timer 2 Initialization with 90-degree phase shift  
 * Channel C (CH3) - Normal polarity
 * Channel D (CH4) - Inverse polarity relative to CH3
 */
void Timer2_Init(void)
{
    /* Enable Timer 2 clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* Configure Timer 2 base */
    LL_TIM_SetPrescaler(TIM2, TIM2_PRESCALER);
    LL_TIM_SetAutoReload(TIM2, 1000);  // Default to 1kHz
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);

    /* Configure Channel 3 (Normal polarity) */
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetCompareCH3(TIM2, 500);  // 50% duty cycle

    /* Configure Channel 4 (Inverse polarity relative to CH3) */
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);  // Inverse polarity
    LL_TIM_OC_SetCompareCH4(TIM2, 500);  // Same compare value as CH3

    /* Enable outputs */
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
}

/**
 * @brief Set 90-degree phase shift between Timer 1 and Timer 2
 */
void Timer_SetPhaseShift(void)
{
    uint32_t period = LL_TIM_GetAutoReload(TIM1);
    uint32_t phase_offset = period / 4;  // 90 degrees = 1/4 period
    
    /* Set Timer 2 counter to create 90-degree phase shift */
    LL_TIM_SetCounter(TIM2, phase_offset);
}

/**
 * @brief ADC Initialization for potentiometer
 */
void ADC_Init(void)
{
    /* Enable ADC clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
    
    /* Configure ADC */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    
    /* Configure regular sequence */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC_POTENTIOMETER_CHANNEL);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_POTENTIOMETER_CHANNEL, LL_ADC_SAMPLINGTIME_41CYCLES_5);
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    /* Wait for ADC ready */
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
    
    /* Start calibration */
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
}

/**
 * @brief Status LED Initialization
 */
void StatusLED_Init(void)
{
    /* Configure LED pin as output */
    LL_GPIO_SetPinMode(STATUS_LED_PORT, STATUS_LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(STATUS_LED_PORT, STATUS_LED_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(STATUS_LED_PORT, STATUS_LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    
    /* LED off initially (active low on Blue Pill) */
    LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);
}

/**
 * @brief Button Initialization with interrupt handling
 */
void Buttons_Init(void)
{
    /* Configure button pins as inputs with pull-up */
    LL_GPIO_SetPinMode(BUTTON_ENABLE_PORT, BUTTON_ENABLE_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(BUTTON_ENABLE_PORT, BUTTON_ENABLE_PIN, LL_GPIO_PULL_UP);
    
    LL_GPIO_SetPinMode(BUTTON_DISABLE_PORT, BUTTON_DISABLE_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(BUTTON_DISABLE_PORT, BUTTON_DISABLE_PIN, LL_GPIO_PULL_UP);
    
    /* Enable EXTI clock */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    
    /* Configure EXTI for buttons (falling edge - button press) */
    LL_EXTI_InitTypeDef exti_init = {0};
    
    /* Enable button */
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE12);
    exti_init.Line_0_31 = LL_EXTI_LINE_12;
    exti_init.LineCommand = ENABLE;
    exti_init.Mode = LL_EXTI_MODE_IT;
    exti_init.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&exti_init);
    
    /* Disable button */
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE15);
    exti_init.Line_0_31 = LL_EXTI_LINE_15;
    LL_EXTI_Init(&exti_init);
    
    /* Enable NVIC */
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief UART Initialization for status output
 */
void UART_Init(void)
{
    /* Enable UART clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    
    /* Configure UART pins - NOTE: These conflict with timer pins, using different pins */
    /* TX: PA2 is used by TIM2_CH3, using different approach for UART */
    /* For now, disable UART to avoid conflicts - can be implemented with different pins */
    
    /* TODO: Implement UART on different pins or use software UART */
}

/**
 * @brief Initialize the complete clock generator system
 */
void ClockGenerator_Init(void)
{
    /* Initialize system state */
    g_system_state.output_enabled = 0;
    g_system_state.current_frequency = DEFAULT_FREQ;
    g_system_state.adc_value = 0;
    g_system_state.last_button_time = 0;
    
    /* Initialize hardware */
    SystemClock_Config();
    GPIO_Init();
    Timer1_Init();
    Timer2_Init();
    ADC_Init();
    StatusLED_Init();
    Buttons_Init();
    
    /* Set initial frequency */
    ClockGenerator_SetFrequency(DEFAULT_FREQ);
}

/**
 * @brief Start the clock generator timers with phase shift
 */
void ClockGenerator_Start(void)
{
    if (!g_system_state.output_enabled) {
        g_system_state.output_enabled = 1;
        
        /* Start Timer 1 first */
        LL_TIM_EnableCounter(TIM1);
        
        /* Small delay to ensure Timer 1 is running */
        for (volatile int i = 0; i < 1000; i++);
        
        /* Start Timer 2 with phase offset */
        Timer_SetPhaseShift();
        LL_TIM_EnableCounter(TIM2);
        
        /* Turn on status LED (active low) */
        LL_GPIO_ResetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

/**
 * @brief Stop the clock generator timers
 */
void ClockGenerator_Stop(void)
{
    if (g_system_state.output_enabled) {
        g_system_state.output_enabled = 0;
        
        LL_TIM_DisableCounter(TIM1);
        LL_TIM_DisableCounter(TIM2);
        
        /* Turn off status LED (active low) */
        LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

/**
 * @brief Set the output frequency for both timers maintaining phase shift
 * @param frequency: Desired frequency in Hz
 */
void ClockGenerator_SetFrequency(uint32_t frequency)
{
    /* Clamp frequency to valid range */
    if (frequency < MIN_OUTPUT_FREQ) frequency = MIN_OUTPUT_FREQ;
    if (frequency > MAX_OUTPUT_FREQ) frequency = MAX_OUTPUT_FREQ;
    
    g_system_state.current_frequency = frequency;
    
    uint32_t auto_reload = (TIM1_FREQUENCY / frequency) - 1;
    uint32_t compare_value = auto_reload / 2;  // 50% duty cycle
    
    /* Update Timer 1 */
    LL_TIM_SetAutoReload(TIM1, auto_reload);
    LL_TIM_OC_SetCompareCH1(TIM1, compare_value);
    LL_TIM_OC_SetCompareCH2(TIM1, compare_value);
    
    /* Update Timer 2 */
    LL_TIM_SetAutoReload(TIM2, auto_reload);
    LL_TIM_OC_SetCompareCH3(TIM2, compare_value);
    LL_TIM_OC_SetCompareCH4(TIM2, compare_value);
    
    /* Maintain phase shift if running */
    if (g_system_state.output_enabled) {
        Timer_SetPhaseShift();
    }
}

/**
 * @brief Read ADC value from potentiometer
 */
uint16_t ADC_ReadPotentiometer(void)
{
    /* Start conversion */
    LL_ADC_REG_StartConversionSWStart(ADC1);
    
    /* Wait for conversion complete */
    while (!LL_ADC_IsActiveFlag_EOC(ADC1));
    
    /* Read and return value */
    return LL_ADC_REG_ReadConversionData12(ADC1);
}

/**
 * @brief Convert ADC value to frequency
 */
uint32_t ADC_ValueToFrequency(uint16_t adc_value)
{
    /* Linear mapping from ADC range to frequency range */
    uint32_t frequency = MIN_OUTPUT_FREQ + 
        ((uint32_t)(adc_value) * (MAX_OUTPUT_FREQ - MIN_OUTPUT_FREQ)) / ADC_MAX_VALUE;
    
    return frequency;
}

/**
 * @brief Get system tick count
 */
uint32_t GetSystemTick(void)
{
    return g_system_state.last_button_time;  // Simplified for now
}

/**
 * @brief Status LED control functions
 */
void StatusLED_On(void)
{
    LL_GPIO_ResetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);  // Active low
}

void StatusLED_Off(void)
{
    LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);  // Active low
}

/**
 * @brief UART functions (simplified for now)
 */
void UART_SendString(const char* str)
{
    /* TODO: Implement UART transmission */
    (void)str;  // Suppress unused parameter warning
}

void UART_SendStatus(const system_state_t* state)
{
    /* TODO: Implement status transmission */
    (void)state;  // Suppress unused parameter warning
}

/**
 * @brief SysTick interrupt handler
 */
void SysTick_Handler(void)
{
    g_system_tick++;
}

/**
 * @brief EXTI interrupt handler for buttons
 */
void EXTI15_10_IRQHandler(void)
{
    uint32_t current_time = g_system_tick;
    
    /* Debounce check */
    if ((current_time - g_system_state.last_button_time) < DEBOUNCE_TIME_MS) {
        /* Clear all EXTI flags */
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12 | LL_EXTI_LINE_15);
        return;
    }
    
    g_system_state.last_button_time = current_time;
    
    /* Check which button was pressed */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET) {
        /* Enable button pressed */
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
        ClockGenerator_Start();
    }
    
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET) {
        /* Disable button pressed */
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
        ClockGenerator_Stop();
    }
}
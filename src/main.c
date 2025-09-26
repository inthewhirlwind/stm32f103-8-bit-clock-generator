#include "clock_generator.h"
#include "stm32f1xx_ll_utils.h"

/**
 * @brief Main application for STM32F103 8-bit Clock Generator
 * 
 * This application generates clock signals using Timer 1 and Timer 2:
 * - Timer 1 Channel A (PA8): Normal polarity PWM
 * - Timer 1 Channel B (PA9): Inverse polarity PWM (relative to Channel A)
 * - Timer 2 Channel C (PA2): Normal polarity PWM  
 * - Timer 2 Channel D (PA3): Inverse polarity PWM (relative to Channel C)
 */

int main(void)
{
    /* Initialize the clock generator system */
    ClockGenerator_Init();
    
    /* Start generating clock signals */
    ClockGenerator_Start();
    
    /* Main application loop */
    while (1)
    {
        /* Optional: Add frequency switching or control logic here */
        
        /* Example: Switch between different frequencies every 2 seconds */
        LL_mDelay(2000);
        ClockGenerator_SetFrequency(1000);  // 1kHz
        
        LL_mDelay(2000);
        ClockGenerator_SetFrequency(5000);  // 5kHz
        
        LL_mDelay(2000);
        ClockGenerator_SetFrequency(10000); // 10kHz
        
        LL_mDelay(2000);
        ClockGenerator_SetFrequency(50000); // 50kHz
    }
}

/**
 * @brief Error handler
 */
void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();
    
    /* Stop timers */
    ClockGenerator_Stop();
    
    /* Infinite loop */
    while (1)
    {
    }
}

/**
 * @brief Assert failed callback
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add their implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    Error_Handler();
}
#endif
#include "clock_generator.h"
#include "stm32f1xx_ll_utils.h"
#include <stdlib.h>

/* External system state - declare it but it's defined in clock_generator.c */
/* extern system_state_t g_system_state; */

/**
 * @brief Main application for STM32F103 8-bit Clock Generator
 * 
 * This application generates quadrature clock signals using Timer 1 and Timer 2:
 * - Timer 1 Channel A (PA8): Normal polarity PWM
 * - Timer 1 Channel B (PA9): Inverse polarity PWM (relative to Channel A)
 * - Timer 2 Channel C (PA2): Normal polarity PWM with 90° phase shift
 * - Timer 2 Channel D (PA3): Inverse polarity PWM (relative to Channel C)
 * 
 * Features:
 * - 90-degree phase shift between Timer 1 and Timer 2 for quadrature output
 * - Potentiometer control via ADC for frequency adjustment (10Hz - 1kHz)
 * - Button controls for enable/disable with debouncing
 * - Status LED indication
 * - UART status output (when implemented)
 */

int main(void)
{
    uint16_t last_adc_value = 0;
    uint32_t adc_update_counter = 0;
    uint32_t uart_update_counter = 0;
    const uint32_t ADC_UPDATE_INTERVAL = 100;   // Update frequency every 100 main loops
    const uint32_t UART_UPDATE_INTERVAL = 1000; // Send UART status every 1000 main loops (~1s)
    
    /* Initialize the complete clock generator system */
    ClockGenerator_Init();
    
    /* Main application loop */
    while (1)
    {
        /* Read potentiometer every ADC_UPDATE_INTERVAL iterations */
        if (++adc_update_counter >= ADC_UPDATE_INTERVAL) {
            adc_update_counter = 0;
            
            /* Read ADC value */
            uint16_t current_adc = ADC_ReadPotentiometer();
            
            /* Check if ADC value changed significantly (hysteresis) */
            if (abs((int)current_adc - (int)last_adc_value) > 20) {
                last_adc_value = current_adc;
                
                /* Convert ADC to frequency and update */
                uint32_t new_frequency = ADC_ValueToFrequency(current_adc);
                ClockGenerator_SetFrequency(new_frequency);
                
                /* Update internal system state (through direct access for now) */
                extern system_state_t g_system_state;
                g_system_state.adc_value = current_adc;
                g_system_state.current_frequency = new_frequency;
            }
        }
        
        /* Send UART status periodically */
        if (++uart_update_counter >= UART_UPDATE_INTERVAL) {
            uart_update_counter = 0;
            
            /* Send current status via UART */
            extern system_state_t g_system_state;
            UART_SendStatus(&g_system_state);
        }
        
        /* Small delay to prevent excessive CPU usage */
        LL_mDelay(1);  // 1ms delay
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
    
    /* Turn off status LED */
    StatusLED_Off();
    
    /* Infinite loop */
    while (1)
    {
        /* Flash LED to indicate error */
        StatusLED_On();
        for (volatile int i = 0; i < 100000; i++);
        StatusLED_Off();
        for (volatile int i = 0; i < 100000; i++);
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

/**
 * @brief Hard fault handler
 */
void HardFault_Handler(void)
{
    Error_Handler();
}

/**
 * @brief Memory management fault handler
 */
void MemManage_Handler(void)
{
    Error_Handler();
}

/**
 * @brief Bus fault handler
 */
void BusFault_Handler(void)
{
    Error_Handler();
}

/**
 * @brief Usage fault handler
 */
void UsageFault_Handler(void)
{
    Error_Handler();
}
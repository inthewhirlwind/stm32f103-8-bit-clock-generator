#ifndef CLOCK_GENERATOR_H
#define CLOCK_GENERATOR_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_exti.h"

/* Timer Configuration */
#define TIM1_FREQUENCY      1000000  // 1MHz base frequency
#define TIM1_PRESCALER      (SystemCoreClock / TIM1_FREQUENCY - 1)
#define TIM2_FREQUENCY      1000000  // 1MHz base frequency  
#define TIM2_PRESCALER      (SystemCoreClock / TIM2_FREQUENCY - 1)

/* Frequency Control */
#define MIN_OUTPUT_FREQ     10      // 10Hz minimum
#define MAX_OUTPUT_FREQ     1000    // 1kHz maximum
#define DEFAULT_FREQ        100     // 100Hz default

/* ADC Configuration */
#define ADC_POTENTIOMETER_CHANNEL   LL_ADC_CHANNEL_0  // PA0
#define ADC_MAX_VALUE               4095
#define ADC_RESOLUTION_BITS         12

/* GPIO Pin Definitions - Timer Outputs */
#define TIM1_CH1_GPIO_PORT  GPIOA
#define TIM1_CH1_GPIO_PIN   LL_GPIO_PIN_8
#define TIM1_CH2_GPIO_PORT  GPIOA
#define TIM1_CH2_GPIO_PIN   LL_GPIO_PIN_9

#define TIM2_CH3_GPIO_PORT  GPIOA
#define TIM2_CH3_GPIO_PIN   LL_GPIO_PIN_2
#define TIM2_CH4_GPIO_PORT  GPIOA
#define TIM2_CH4_GPIO_PIN   LL_GPIO_PIN_3

/* GPIO Pin Definitions - Control Inputs */
#define BUTTON_ENABLE_PORT  GPIOA
#define BUTTON_ENABLE_PIN   LL_GPIO_PIN_12
#define BUTTON_DISABLE_PORT GPIOA
#define BUTTON_DISABLE_PIN  LL_GPIO_PIN_15

/* GPIO Pin Definitions - Status Output */
#define STATUS_LED_PORT     GPIOC
#define STATUS_LED_PIN      LL_GPIO_PIN_13

/* GPIO Pin Definitions - ADC Input */
#define POTENTIOMETER_PORT  GPIOA
#define POTENTIOMETER_PIN   LL_GPIO_PIN_0

/* UART Configuration */
#define UART_PORT           USART2
#define UART_BAUD_RATE      115200
#define UART_TX_PORT        GPIOA
#define UART_TX_PIN         LL_GPIO_PIN_2
#define UART_RX_PORT        GPIOA
#define UART_RX_PIN         LL_GPIO_PIN_3

/* Debounce Configuration */
#define DEBOUNCE_TIME_MS    50

/* System State */
typedef struct {
    uint8_t output_enabled;
    uint32_t current_frequency;
    uint16_t adc_value;
    uint32_t last_button_time;
} system_state_t;

/* Function Prototypes - Core System */
void SystemClock_Config(void);
void ClockGenerator_Init(void);
void ClockGenerator_Start(void);
void ClockGenerator_Stop(void);
void ClockGenerator_SetFrequency(uint32_t frequency);

/* Function Prototypes - Peripheral Initialization */
void GPIO_Init(void);
void Timer1_Init(void);
void Timer2_Init(void);
void ADC_Init(void);
void UART_Init(void);

/* Function Prototypes - Control Functions */
void Buttons_Init(void);
void Button_Enable_Interrupt(void);
void Button_Disable_Interrupt(void);
void StatusLED_Init(void);
void StatusLED_On(void);
void StatusLED_Off(void);

/* Function Prototypes - ADC Functions */
uint16_t ADC_ReadPotentiometer(void);
uint32_t ADC_ValueToFrequency(uint16_t adc_value);

/* Function Prototypes - UART Functions */
void UART_SendString(const char* str);
void UART_SendStatus(const system_state_t* state);

/* Function Prototypes - Timer Control with Phase Shift */
void Timer_SetPhaseShift(void);
uint32_t GetSystemTick(void);

#endif /* CLOCK_GENERATOR_H */
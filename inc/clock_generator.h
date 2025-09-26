#ifndef CLOCK_GENERATOR_H
#define CLOCK_GENERATOR_H

#include "stm32f1xx.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"

/* Timer 1 Configuration */
#define TIM1_FREQUENCY      1000000  // 1MHz base frequency
#define TIM1_PRESCALER      (SystemCoreClock / TIM1_FREQUENCY - 1)

/* Timer 2 Configuration */
#define TIM2_FREQUENCY      1000000  // 1MHz base frequency  
#define TIM2_PRESCALER      (SystemCoreClock / TIM2_FREQUENCY - 1)

/* GPIO Pin Definitions */
#define TIM1_CH1_GPIO_PORT  GPIOA
#define TIM1_CH1_GPIO_PIN   LL_GPIO_PIN_8
#define TIM1_CH2_GPIO_PORT  GPIOA
#define TIM1_CH2_GPIO_PIN   LL_GPIO_PIN_9

#define TIM2_CH3_GPIO_PORT  GPIOA
#define TIM2_CH3_GPIO_PIN   LL_GPIO_PIN_2
#define TIM2_CH4_GPIO_PORT  GPIOA
#define TIM2_CH4_GPIO_PIN   LL_GPIO_PIN_3

/* Function Prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void Timer1_Init(void);
void Timer2_Init(void);
void ClockGenerator_Init(void);
void ClockGenerator_Start(void);
void ClockGenerator_Stop(void);
void ClockGenerator_SetFrequency(uint32_t frequency);

#endif /* CLOCK_GENERATOR_H */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Pin Definitions */
// Quadrature output pins (using Timer PWM outputs)
#define QUAD_OUT_A_Pin GPIO_PIN_8
#define QUAD_OUT_A_GPIO_Port GPIOA
#define QUAD_OUT_B_Pin GPIO_PIN_11
#define QUAD_OUT_B_GPIO_Port GPIOA
#define QUAD_OUT_C_Pin GPIO_PIN_6
#define QUAD_OUT_C_GPIO_Port GPIOB
#define QUAD_OUT_D_Pin GPIO_PIN_7
#define QUAD_OUT_D_GPIO_Port GPIOB

// Potentiometer analog input (ADC)
#define POT_ADC_Pin GPIO_PIN_0
#define POT_ADC_GPIO_Port GPIOA

// Button inputs
#define ENABLE_BTN_Pin GPIO_PIN_2
#define ENABLE_BTN_GPIO_Port GPIOA
#define DISABLE_BTN_Pin GPIO_PIN_3
#define DISABLE_BTN_GPIO_Port GPIOA

// Status LED
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC

// UART pins (USART1)
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA

/* Frequency control constants */
#define MIN_FREQUENCY_HZ 10
#define MAX_FREQUENCY_HZ 1000
#define ADC_MAX_VALUE 4095
#define UART_BAUD_RATE 115200

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
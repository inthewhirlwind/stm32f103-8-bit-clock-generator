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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC
#define ADC_POT_Pin GPIO_PIN_0
#define ADC_POT_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define QUAD_OUT_A_Pin GPIO_PIN_8
#define QUAD_OUT_A_GPIO_Port GPIOA
#define QUAD_OUT_B_Pin GPIO_PIN_9
#define QUAD_OUT_B_GPIO_Port GPIOA
#define QUAD_OUT_C_Pin GPIO_PIN_10
#define QUAD_OUT_C_GPIO_Port GPIOA
#define QUAD_OUT_D_Pin GPIO_PIN_11
#define QUAD_OUT_D_GPIO_Port GPIOA
#define BUTTON_ENABLE_Pin GPIO_PIN_12
#define BUTTON_ENABLE_GPIO_Port GPIOA
#define BUTTON_DISABLE_Pin GPIO_PIN_15
#define BUTTON_DISABLE_GPIO_Port GPIOA

/* Private defines -----------------------------------------------------------*/
#define FREQ_MIN_HZ 10
#define FREQ_MAX_HZ 1000
#define ADC_MAX_VALUE 4095

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
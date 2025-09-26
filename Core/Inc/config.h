/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Configuration header for STM32F103 Quadrature Clock Generator
  ******************************************************************************
  * @attention
  *
  * This file contains user-configurable parameters for the quadrature clock
  * generator project. Modify these values to customize the behavior.
  *
  ******************************************************************************
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Frequency Configuration */
#define MIN_FREQUENCY_HZ            10      /* Minimum output frequency in Hz */
#define MAX_FREQUENCY_HZ            1000    /* Maximum output frequency in Hz */

/* ADC Configuration */
#define ADC_MAX_VALUE               4095    /* 12-bit ADC maximum value */
#define ADC_FREQUENCY_DEADBAND      5       /* Hz deadband to prevent jitter */

/* UART Configuration */
#define UART_BAUD_RATE              115200  /* UART baud rate */
#define UART_STATUS_INTERVAL_MS     500     /* Status message interval */

/* Button Configuration */
#define BUTTON_DEBOUNCE_MS          200     /* Button debounce delay */

/* System Configuration */
#define SYSTEM_CLOCK_MHZ            72      /* System clock frequency */
#define TIMER_CLOCK_MHZ             1       /* Timer clock after prescaler */

/* Pin Assignments - Modify if using different pins */

/* Quadrature Outputs */
#define QUAD_A_PORT                 GPIOA
#define QUAD_A_PIN                  GPIO_PIN_8
#define QUAD_A_TIMER                TIM1
#define QUAD_A_CHANNEL              TIM_CHANNEL_1

#define QUAD_B_PORT                 GPIOA  
#define QUAD_B_PIN                  GPIO_PIN_11
#define QUAD_B_TIMER                TIM1
#define QUAD_B_CHANNEL              TIM_CHANNEL_4

#define QUAD_C_PORT                 GPIOB
#define QUAD_C_PIN                  GPIO_PIN_6
#define QUAD_C_TIMER                TIM4
#define QUAD_C_CHANNEL              TIM_CHANNEL_1

#define QUAD_D_PORT                 GPIOB
#define QUAD_D_PIN                  GPIO_PIN_7
#define QUAD_D_TIMER                TIM4
#define QUAD_D_CHANNEL              TIM_CHANNEL_2

/* Analog Input */
#define POT_ADC_PORT                GPIOA
#define POT_ADC_PIN                 GPIO_PIN_0
#define POT_ADC_CHANNEL             ADC_CHANNEL_0

/* Digital Inputs */
#define ENABLE_BTN_PORT             GPIOA
#define ENABLE_BTN_PIN              GPIO_PIN_2

#define DISABLE_BTN_PORT            GPIOA
#define DISABLE_BTN_PIN             GPIO_PIN_3

/* Status LED */
#define STATUS_LED_PORT             GPIOC
#define STATUS_LED_PIN              GPIO_PIN_13

/* UART Interface */
#define UART_TX_PORT                GPIOA
#define UART_TX_PIN                 GPIO_PIN_9

#define UART_RX_PORT                GPIOA
#define UART_RX_PIN                 GPIO_PIN_10

/* Version Information */
#define FIRMWARE_VERSION_MAJOR      1
#define FIRMWARE_VERSION_MINOR      0
#define FIRMWARE_VERSION_PATCH      0

#define FIRMWARE_VERSION_STRING     "1.0.0"
#define PROJECT_NAME                "STM32F103 Quadrature Clock Generator"

/* Debug Configuration */
#define DEBUG_ENABLED               1       /* Enable debug features */
#define VERBOSE_UART_OUTPUT         1       /* Enable verbose UART messages */

/* Safety Limits */
#define MAX_TIMER_PERIOD            65535   /* Maximum timer period value */
#define MIN_TIMER_PERIOD            100     /* Minimum timer period value */

/* Calculated Values - Do not modify */
#define TIMER_PRESCALER             (SYSTEM_CLOCK_MHZ - 1)  /* For 1MHz timer clock */
#define FREQUENCY_RANGE             (MAX_FREQUENCY_HZ - MIN_FREQUENCY_HZ)

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
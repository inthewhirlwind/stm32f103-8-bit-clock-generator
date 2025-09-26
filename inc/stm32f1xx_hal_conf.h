#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#define HAL_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    8000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  40000U    
#endif /* LSI_VALUE */

/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  32768U    
#endif /* LSE_VALUE */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */     
#define  VDD_VALUE                    3300U /*!< Value of VDD in mv */           
#define  TICK_INT_PRIORITY            0x0FU /*!< tick interrupt priority */            
#define  USE_RTOS                     0U     
#define  PREFETCH_ENABLE              1U              
#define  USE_SPI_CRC                  1U     

#define  USE_HAL_TIM_REGISTER_CALLBACKS  0U /* TIM register callback disabled  */

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/*#define USE_FULL_ASSERT    1U*/

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

#ifdef HAL_RCC_MODULE_ENABLED
 #include "stm32f1xx_ll_rcc.h"
 #include "stm32f1xx_ll_bus.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
 #include "stm32f1xx_ll_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f1xx_ll_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
 #include "stm32f1xx_ll_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f1xx_ll_usart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
 #include "stm32f1xx_ll_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
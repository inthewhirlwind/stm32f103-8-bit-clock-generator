#include "stm32f1xx.h"

/* System Core Clock variable */
uint32_t SystemCoreClock = 8000000;

/* AHB prescaler table */
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

/* APB prescaler table */
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

/**
 * @brief Setup the microcontroller system
 */
void SystemInit(void)
{
    /* FPU settings (not applicable for STM32F103) */
    
    /* Reset the RCC clock configuration to the default reset state */
    RCC->CR |= 0x00000001U;
    
    /* Reset CFGR register */
    RCC->CFGR &= 0xF8FF0000U;
    
    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFFU;
    
    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFU;
    
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
    RCC->CFGR &= 0xFF80FFFFU;
    
    /* Disable all interrupts and clear pending bits */
    RCC->CIR = 0x009F0000U;
}

/**
 * @brief Update SystemCoreClock variable according to Clock Register Values
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;
    
    /* Get SYSCLK source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (tmp)
    {
        case 0x00U:  /* HSI used as system clock source */
            SystemCoreClock = HSI_VALUE;
            break;
        case 0x04U:  /* HSE used as system clock source */
            SystemCoreClock = HSE_VALUE;
            break;
        case 0x08U:  /* PLL used as system clock source */
            /* Get PLL clock source and multiplication factor */
            pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
            
            pllmull = (pllmull >> 18U) + 2U;
            
            if (pllsource == 0x00U)
            {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
            }
            else
            {
                /* HSE selected as PLL clock entry */
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
                {/* HSE oscillator clock divided by 2 */
                    SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
                }
                else
                {
                    SystemCoreClock = HSE_VALUE * pllmull;
                }
            }
            break;
            
        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }
    
    /* Compute HCLK frequency */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    SystemCoreClock >>= tmp;
}
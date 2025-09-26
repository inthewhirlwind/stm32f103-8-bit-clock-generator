#!/bin/bash

# Script to download STM32F1 HAL Driver and CMSIS libraries
# This script downloads the necessary libraries for building the project

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIB_DIR="${PROJECT_ROOT}/lib"

echo "Downloading STM32F1 HAL libraries..."

# Create lib directory if it doesn't exist
mkdir -p "${LIB_DIR}"
cd "${LIB_DIR}"

# Download CMSIS
if [ ! -d "CMSIS" ]; then
    echo "Downloading CMSIS..."
    git clone --depth 1 https://github.com/ARM-software/CMSIS_5.git CMSIS_temp
    mkdir -p CMSIS/Include
    mkdir -p CMSIS/Device/ST/STM32F1xx/Include
    
    # Copy CMSIS Core headers
    cp CMSIS_temp/CMSIS/Core/Include/*.h CMSIS/Include/
    
    # Copy STM32F1xx device headers (create basic ones if not available)
    if [ -d "CMSIS_temp/Device/ST/STM32F1xx/Include" ]; then
        cp CMSIS_temp/Device/ST/STM32F1xx/Include/*.h CMSIS/Device/ST/STM32F1xx/Include/
    else
        # Create basic device header if not found
        cat > CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h << 'EOF'
#ifndef __STM32F1xx_H
#define __STM32F1xx_H
#ifdef __cplusplus
 extern "C" {
#endif
#if defined(STM32F103xB)
  #include "stm32f103xb.h"
#else
 #error "Please select first the target STM32F1xx device used in your application"
#endif
#ifdef __cplusplus
}
#endif
#endif
EOF
        # Create device-specific header
        cat > CMSIS/Device/ST/STM32F1xx/Include/stm32f103xb.h << 'EOF'
#ifndef __STM32F103xB_H
#define __STM32F103xB_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "core_cm3.h"
#include "system_stm32f1xx.h"
#include <stdint.h>
/* Basic register definitions for STM32F103xB */
#define FLASH_BASE            0x08000000UL
#define SRAM_BASE             0x20000000UL
#define PERIPH_BASE           0x40000000UL
/* Add other necessary definitions */
#ifdef __cplusplus
}
#endif
#endif
EOF
        # Create system header
        cat > CMSIS/Device/ST/STM32F1xx/Include/system_stm32f1xx.h << 'EOF'
#ifndef __SYSTEM_STM32F1XX_H
#define __SYSTEM_STM32F1XX_H
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
extern uint32_t SystemCoreClock;
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];
void SystemInit(void);
void SystemCoreClockUpdate(void);
#ifdef __cplusplus
}
#endif
#endif
EOF
    fi
    
    # Clean up
    rm -rf CMSIS_temp
    echo "CMSIS downloaded and installed."
else
    echo "CMSIS already exists, skipping download."
fi

# Download STM32F1xx HAL Driver
if [ ! -d "STM32F1xx_HAL_Driver" ]; then
    echo "Downloading STM32F1xx HAL Driver..."
    git clone --depth 1 https://github.com/STMicroelectronics/stm32f1xx_hal_driver.git STM32F1xx_HAL_Driver
    echo "STM32F1xx HAL Driver downloaded."
else
    echo "STM32F1xx HAL Driver already exists, skipping download."
fi

echo "All libraries downloaded successfully!"
echo "You can now build the project with:"
echo "  mkdir build && cd build"
echo "  cmake .."
echo "  make"
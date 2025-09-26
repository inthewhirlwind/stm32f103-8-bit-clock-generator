cmake_minimum_required(VERSION 3.20)

# Set toolchain file before project() call
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

# Project configuration
project(stm32f103-quadrature-generator 
    VERSION 1.0.0
    DESCRIPTION "STM32F103C8T6 Quadrature Clock Generator"
    LANGUAGES C ASM
)

# MCU Configuration
set(MCU_FAMILY STM32F1xx)
set(MCU_MODEL STM32F103xB)
set(CPU_PARAMETERS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")

# HAL and CMSIS library paths
set(CMSIS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/CMSIS)
set(HAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32F1xx_HAL_Driver)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMSIS_PATH}/Include
    ${CMSIS_PATH}/Device/ST/STM32F1xx/Include
    ${HAL_PATH}/Inc
    ${HAL_PATH}/Inc/Legacy
)

# Compiler definitions
add_definitions(
    -DSTM32F103xB
    -DUSE_HAL_DRIVER
    -DHSE_VALUE=8000000
)

# Compiler flags
set(CMAKE_C_FLAGS "${CPU_PARAMETERS} -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -gdwarf-2")

# Different build types
set(CMAKE_C_FLAGS_DEBUG "-Og -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CPU_PARAMETERS} -Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")

# Source files
file(GLOB_RECURSE SOURCES
    "src/*.c"
    "src/*.s"
)

# HAL Source files
file(GLOB HAL_SOURCES
    "${HAL_PATH}/Src/stm32f1xx_hal.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_adc.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_adc_ex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_cortex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_dma.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_flash.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_flash_ex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_gpio.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_gpio_ex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_pwr.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_rcc.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_rcc_ex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_tim.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_tim_ex.c"
    "${HAL_PATH}/Src/stm32f1xx_hal_uart.c"
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HAL_SOURCES})

# Post build commands to create hex and bin files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Building HEX and BIN files"
)

# Print executable size
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
)
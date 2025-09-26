cmake_minimum_required(VERSION 3.16)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define target processor
set(MCU_FAMILY STM32F1)
set(MCU_MODEL STM32F103xB)

# Project
project(stm32f103-clock-generator VERSION 1.0.0 LANGUAGES C ASM)

# STM32 HAL and CMSIS files
set(STM32_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver)
set(STM32_CMSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${STM32_HAL_DIR}/Inc
    ${STM32_HAL_DIR}/Inc/Legacy
    ${STM32_CMSIS_DIR}/Device/ST/STM32F1xx/Include
    ${STM32_CMSIS_DIR}/Include
)

# Define symbols
add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xB)

# HAL source files
file(GLOB_RECURSE HAL_SOURCES 
    ${STM32_HAL_DIR}/Src/*.c
)

# Remove template files that cause build issues
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*template.*")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*msp.*")

# CMSIS startup file
set(STARTUP_FILE ${STM32_CMSIS_DIR}/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s)

# Core source files
file(GLOB_RECURSE CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c
)

# Linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections")

# Create executable
add_executable(${PROJECT_NAME} ${CORE_SOURCES} ${HAL_SOURCES} ${STARTUP_FILE})

# Print executable size
add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${SIZE} ${PROJECT_NAME}
)

# Create hex file
add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)

# Create binary file
add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMENT "Generating BIN file"
)
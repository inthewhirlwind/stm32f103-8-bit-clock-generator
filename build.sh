#!/bin/bash

# STM32F103 Quadrature Clock Generator Build Script
# This script automates the build process for the project

set -e  # Exit on any error

echo "STM32F103 Quadrature Clock Generator Build Script"
echo "================================================="

# Check if ARM toolchain is installed
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "Error: ARM GCC toolchain not found!"
    echo "Please install arm-none-eabi-gcc and cmake"
    echo "Ubuntu/Debian: sudo apt install gcc-arm-none-eabi cmake"
    exit 1
fi

# Check if cmake is installed
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake not found!"
    echo "Please install cmake"
    exit 1
fi

echo "Toolchain versions:"
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
echo ""

# Clean previous build
if [ -d "build" ]; then
    echo "Cleaning previous build..."
    rm -rf build
fi

# Create build directory
echo "Creating build directory..."
mkdir build
cd build

# Configure project
echo "Configuring project with CMake..."
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake ..

# Build project
echo "Building project..."
make -j$(nproc)

# Generate hex and binary files
echo "Generating hex and binary files..."
arm-none-eabi-objcopy -O ihex stm32f103-clock-generator.elf stm32f103-clock-generator.hex
arm-none-eabi-objcopy -O binary stm32f103-clock-generator.elf stm32f103-clock-generator.bin

# Show memory usage
echo ""
echo "Memory Usage:"
echo "============="
arm-none-eabi-size stm32f103-clock-generator.elf

# Show generated files
echo ""
echo "Generated Files:"
echo "================"
ls -la *.elf *.hex *.bin *.map 2>/dev/null | awk '{print $9, $5" bytes"}' || echo "Error: Build artifacts not found"

echo ""
echo "Build completed successfully!"
echo "Flash the firmware using:"
echo "  st-flash write stm32f103-clock-generator.bin 0x8000000"
echo "or"
echo "  STM32_Programmer_CLI -c port=swd -w stm32f103-clock-generator.hex -v -rst"
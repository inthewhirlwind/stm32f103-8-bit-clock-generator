# STM32F103 8-bit Clock Generator

An 8-bit microprocessor system clock generator using an STM32F103 bluepill development board.

## Features

- **Wide frequency range**: 1Hz to 100kHz output frequency
- **Dual-range potentiometer mapping** for precise control across the entire frequency spectrum
- **STM32F103 Bluepill based** - affordable and widely available
- **PWM output** with 50% duty cycle for clean clock signal
- **Real-time frequency adjustment** via potentiometer input

## Frequency Mapping

The potentiometer input is mapped to two distinct frequency ranges to provide both fine control at low frequencies and access to the full frequency spectrum:

### Range 1: Low Frequencies (First 20% of potentiometer rotation)
- **ADC Range**: 0 - 819 (20% of 12-bit ADC)
- **Frequency Range**: 1Hz - 100Hz (linear mapping)
- **Use Case**: Precise control for very low frequency clock signals

### Range 2: High Frequencies (Remaining 80% of potentiometer rotation)  
- **ADC Range**: 820 - 4095 (remaining 80% of 12-bit ADC)
- **Frequency Range**: 100Hz - 100kHz (linear mapping)
- **Use Case**: Full spectrum coverage for higher frequency applications

## Hardware Connections

### STM32F103 Bluepill Pinout
- **PA0**: Potentiometer input (ADC1_IN0) - Connect to potentiometer wiper
- **PA1**: Clock output (TIM2_CH2) - Connect to target system clock input
- **3.3V**: Potentiometer VCC
- **GND**: Potentiometer GND and common ground

### Potentiometer Connection
```
Potentiometer (10kΩ recommended)
├─ Pin 1: Connect to 3.3V
├─ Pin 2: Connect to PA0 (ADC input)
└─ Pin 3: Connect to GND
```

## Building and Programming

### Prerequisites
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- STM32 HAL libraries
- ST-Link programmer or compatible

### Build Commands
```bash
# Build the project
make all

# View frequency mapping table
make frequency_table

# Clean build files
make clean

# Show test frequencies for validation
make test_frequencies
```

### Programming
Use your preferred STM32 programming tool (ST-Link, OpenOCD, etc.) to flash the generated `.hex` or `.bin` file to the STM32F103.

## Testing

The project includes comprehensive test functions to validate the frequency mapping:

```bash
# The following test points should be verified:
# ADC 0    → 1 Hz
# ADC 410  → ~50 Hz  
# ADC 819  → 100 Hz
# ADC 820  → 100 Hz
# ADC 2457 → ~50 kHz
# ADC 4095 → 100 kHz
```

## Configuration

Key parameters can be modified in `inc/config.h`:
- Frequency ranges and thresholds
- GPIO pin assignments  
- Timer and ADC configurations
- Update intervals

## Technical Details

- **Microcontroller**: STM32F103C8T6 (72MHz ARM Cortex-M3)
- **ADC Resolution**: 12-bit (4096 steps)
- **Timer Resolution**: 1MHz base frequency with dynamic period adjustment
- **Output**: PWM signal with 50% duty cycle
- **Update Rate**: 100Hz potentiometer sampling

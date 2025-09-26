# STM32F103 Quadrature Clock Generator

A configurable quadrature square wave generator for the STM32F103C8T6 microcontroller. This project generates 4 synchronized square wave outputs with 90-degree phase differences, making it ideal for driving stepper motors, encoders, or other applications requiring quadrature signals.

## Features

- **Quadrature Output Generation**: 4 square wave outputs with 90-degree phase shifts
- **Variable Frequency**: Adjustable from 10Hz to 1kHz via analog potentiometer
- **50% Duty Cycle**: Precise 50% duty cycle using hardware timers
- **Button Control**: Enable/disable output via dedicated buttons
- **Status LED**: Visual indication of output state
- **UART Communication**: Real-time status output via serial interface
- **Synchronized Timers**: Uses coupled hardware timers for precise phase relationships

## Hardware Requirements

### STM32F103C8T6 Pin Configuration

| Function | Pin | Description |
|----------|-----|-------------|
| Quadrature Output A | PA8 | TIM1_CH1 - 0° phase |
| Quadrature Output B | PA11 | TIM1_CH4 - 90° phase |
| Quadrature Output C | PB6 | TIM4_CH1 - 180° phase |
| Quadrature Output D | PB7 | TIM4_CH2 - 270° phase |
| Potentiometer Input | PA0 | ADC1_IN0 - Frequency control |
| Enable Button | PA2 | Digital input (pullup) |
| Disable Button | PA3 | Digital input (pullup) |
| Status LED | PC13 | Output indicator |
| UART TX | PA9 | USART1_TX - Status output |
| UART RX | PA10 | USART1_RX - Serial input |

### External Components

- **Potentiometer**: 10kΩ linear potentiometer connected between 3.3V and GND, wiper to PA0
- **Buttons**: Two momentary push buttons (active low) with external pullup resistors
- **LED**: Status LED with current limiting resistor on PC13
- **UART Interface**: USB-to-serial converter or direct connection to host

## Software Architecture

### Timer Configuration

- **TIM1**: Master timer generating outputs A (0°) and B (90°)
- **TIM4**: Slave timer generating outputs C (180°) and D (270°)
- **Synchronization**: TIM4 triggered by TIM1 for precise phase alignment
- **Frequency Range**: 10Hz - 1kHz with 1MHz timer clock

### ADC Configuration

- **Channel**: ADC1_IN0 (PA0)
- **Resolution**: 12-bit (4096 steps)
- **Sampling**: Continuous conversion mode
- **Reference**: 3.3V VDD

### UART Configuration

- **Baud Rate**: 115200 bps
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Output**: Status messages every 500ms

## Building the Project

### Prerequisites

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- CMake (version 3.16 or higher)
- Make

### Build Instructions

```bash
# Clone the repository
git clone <repository-url>
cd stm32f103-8-bit-clock-generator

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake ..

# Build the project
make -j4

# Generated files:
# - stm32f103-clock-generator.elf (ELF executable)
# - stm32f103-clock-generator.hex (Intel HEX format)
# - stm32f103-clock-generator.bin (Raw binary)
# - stm32f103-clock-generator.map (Memory map)
```

### Memory Usage

```
   text    data     bss     dec     hex filename
  43296    1744    2748   47788    baac stm32f103-clock-generator.elf
```

- **Flash Usage**: 45,040 bytes (69% of 64KB)
- **RAM Usage**: 4,492 bytes (22% of 20KB)

## Programming the MCU

### Using STM32CubeProgrammer

```bash
# Connect ST-LINK and program the hex file
STM32_Programmer_CLI -c port=swd -w stm32f103-clock-generator.hex -v -rst
```

### Using st-flash

```bash
# Program the binary file
st-flash write stm32f103-clock-generator.bin 0x8000000
```

## Operation

### Startup Sequence

1. System initialization and peripheral configuration
2. Display startup message via UART
3. Enter main control loop

### Main Loop Functions

1. **ADC Reading**: Continuously sample potentiometer value
2. **Frequency Calculation**: Convert ADC value to frequency (10Hz-1kHz)
3. **Button Monitoring**: Check enable/disable button states
4. **Output Control**: Start/stop quadrature generation
5. **Status Reporting**: Send status via UART every 500ms

### UART Output Format

```
STM32F103 Quadrature Clock Generator v1.0
Frequency Range: 10 Hz - 1000 Hz
Status: ENABLED | Freq: 250 Hz | ADC: 1024 (0.82V)
Status: DISABLED | Freq: 500 Hz | ADC: 2048 (1.65V)
```

## Quadrature Signal Timing

The four outputs maintain precise 90-degree phase relationships:

```
Output A: ‾‾‾‾____‾‾‾‾____
Output B: ___‾‾‾‾____‾‾‾‾_
Output C: ____‾‾‾‾____‾‾‾‾
Output D: _‾‾‾‾____‾‾‾‾____
```

## Applications

- **Stepper Motor Driving**: Four-phase stepper motor control
- **Encoder Simulation**: Generate quadrature signals for testing
- **Clock Generation**: Precise timing signals for digital systems
- **Signal Processing**: Test and calibration equipment

## Error Handling

- **Button Debouncing**: 200ms delay prevents false triggers
- **ADC Filtering**: Frequency updates only on significant changes (>5Hz)
- **Error States**: Status LED rapid blinking indicates system error
- **Watchdog**: System reset on critical failures

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with hardware
5. Submit a pull request

## License

This project is released under the MIT License. See LICENSE file for details.

## Support

For questions, issues, or contributions, please open an issue on the GitHub repository.
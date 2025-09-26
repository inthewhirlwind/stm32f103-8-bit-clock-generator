# STM32F103 8-bit Clock Generator

An 8-bit microprocessor system clock generator using an STM32F103 BluePill board with inverse polarity timer channels.

## Features

- **Timer 1 Configuration**: 
  - Channel A (PA8): Normal polarity PWM output
  - Channel B (PA9): **Inverse polarity** PWM output (complementary to Channel A)
- **Timer 2 Configuration**:
  - Channel C (PA2): Normal polarity PWM output  
  - Channel D (PA3): **Inverse polarity** PWM output (complementary to Channel C)
- Configurable output frequencies
- Hardware-generated complementary clock signals
- Low jitter, precise timing
- Optimized for 8-bit microprocessor systems

## Hardware Connections

| Signal | STM32 Pin | Function | Polarity |
|--------|-----------|----------|----------|
| TIM1_CH1 | PA8 | Timer 1 Channel A | Normal |
| TIM1_CH2 | PA9 | Timer 1 Channel B | **Inverse** |
| TIM2_CH3 | PA2 | Timer 2 Channel C | Normal |
| TIM2_CH4 | PA3 | Timer 2 Channel D | **Inverse** |

## Building

```bash
# Full build (requires ARM GCC toolchain)
make

# Syntax check only
make check

# Build simulation
make simulate

# Clean
make clean
```

## Testing

```bash
cd tests
make run_tests
```

## Documentation

See [Inverse Polarity Implementation](docs/INVERSE_POLARITY_IMPLEMENTATION.md) for detailed technical documentation.

## Signal Behavior

The inverse polarity channels generate complementary PWM signals:
- When normal channel outputs HIGH, inverse channel outputs LOW
- When normal channel outputs LOW, inverse channel outputs HIGH
- Perfect 180° phase relationship for clock generation applications

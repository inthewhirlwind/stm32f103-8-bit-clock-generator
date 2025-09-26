# Frequency Mapping Documentation

## Overview

The STM32F103 8-bit clock generator implements a sophisticated dual-range frequency mapping system that provides both precise control at low frequencies and access to the full frequency spectrum up to 100kHz.

## Mapping Algorithm

### Mathematical Implementation

The potentiometer input is read via a 12-bit ADC, providing values from 0 to 4095. This range is divided into two segments:

#### Range 1: Low Frequencies (First 20% of rotation)
- **ADC Input**: 0 to 819 (exactly 20% of 4095)
- **Output Frequency**: 1Hz to 100Hz (linear mapping)
- **Formula**: `frequency = 1 + (adc_value * 99) / 819`

#### Range 2: High Frequencies (Remaining 80% of rotation)  
- **ADC Input**: 820 to 4095 (remaining 80% of range)
- **Output Frequency**: 100Hz to 100kHz (linear mapping)
- **Formula**: `frequency = 100 + ((adc_value - 820) * 99900) / 3275`

### Key Design Points

1. **Smooth Transition**: Both ranges meet exactly at 100Hz (ADC values 819 and 820)
2. **Linear Mapping**: Each range provides linear frequency response within its segment
3. **Maximum Precision**: The low range provides 819 steps across only 99Hz, giving excellent resolution
4. **Full Coverage**: The high range covers the remaining 99.9kHz across 3275 steps

## Test Points and Validation

### Critical Test Points
| ADC Value | Expected Frequency | Description |
|-----------|-------------------|-------------|
| 0         | 1 Hz              | Minimum frequency |
| 410       | ~50 Hz            | Mid-point of low range |
| 819       | 100 Hz            | End of low range |
| 820       | 100 Hz            | Start of high range |
| 2457      | ~50 kHz           | Mid-point of high range |
| 4095      | 100 kHz           | Maximum frequency |

### Boundary Conditions
- **ADC 818**: 99 Hz (just below transition)
- **ADC 821**: ~130 Hz (just above transition)

## Implementation Details

### Hardware Configuration
- **ADC Input**: PA0 (ADC1_IN0) - Connected to potentiometer wiper
- **Clock Output**: PA1 (TIM2_CH2) - PWM output with 50% duty cycle
- **ADC Resolution**: 12-bit (4096 steps)
- **Timer Resolution**: 1MHz base frequency with dynamic period calculation

### Software Architecture
```c
uint32_t map_potentiometer_to_frequency(uint16_t adc_value)
{
    if (adc_value <= LOW_RANGE_THRESHOLD) {
        // Low frequency range: 1-100Hz
        return MIN_FREQUENCY_HZ + 
               (adc_value * (LOW_RANGE_MAX_HZ - MIN_FREQUENCY_HZ)) / 
               LOW_RANGE_THRESHOLD;
    } else {
        // High frequency range: 100Hz-100kHz
        if (adc_value == ADC_MAX_VALUE) {
            return MAX_FREQUENCY_HZ; // Ensure exact maximum
        }
        uint32_t adjusted_adc = adc_value - LOW_RANGE_THRESHOLD - 1;
        uint32_t high_range = ADC_MAX_VALUE - LOW_RANGE_THRESHOLD;
        return LOW_RANGE_MAX_HZ + 
               (adjusted_adc * (MAX_FREQUENCY_HZ - LOW_RANGE_MAX_HZ)) / 
               high_range;
    }
}
```

## Benefits of Dual-Range Design

### Low Frequency Range (1-100Hz)
- **High Resolution**: 819 ADC steps for 99Hz range = ~0.12Hz per step
- **Precise Control**: Critical for applications requiring exact low frequencies
- **Applications**: Slow clock signals, timing references, debugging

### High Frequency Range (100Hz-100kHz)
- **Wide Coverage**: Spans three decades of frequency
- **Adequate Resolution**: 3275 steps for 99.9kHz = ~30.5Hz per step
- **Applications**: System clocks, fast timing signals, general purpose

## Configuration

All frequency mapping parameters are configurable in `inc/config.h`:

```c
#define MIN_FREQUENCY_HZ        1       // Minimum output frequency
#define LOW_RANGE_MAX_HZ        100     // Transition point
#define MAX_FREQUENCY_HZ        100000  // Maximum output frequency
#define LOW_RANGE_PERCENTAGE    20      // Percentage for low range
```

## Testing and Validation

The implementation includes comprehensive test suites:

1. **Unit Tests**: Validate mapping accuracy at key points
2. **Boundary Tests**: Verify smooth transition between ranges
3. **Timer Tests**: Confirm correct period calculations
4. **Integration Tests**: End-to-end frequency generation validation

### Running Tests
```bash
# Compile and run frequency mapping tests
make clean
gcc -DUNIT_TEST_MODE -I inc test_runner.c -o test_runner
./test_runner

# View mapping table
make frequency_table
```

This dual-range approach ensures that users can achieve both the precision needed for low-frequency applications and the wide range required for general-purpose clock generation.
# STM32F103 Clock Generator - Inverse Polarity Implementation

## Overview

This document describes the implementation of inverse polarity timer channels for the STM32F103 8-bit clock generator project.

## Timer Configuration

### Timer 1 (TIM1) - Advanced Control Timer
- **Channel A (CH1) - PA8**: Normal polarity PWM output
- **Channel B (CH2) - PA9**: Inverse polarity PWM output (complementary to CH1)

### Timer 2 (TIM2) - General Purpose Timer  
- **Channel C (CH3) - PA2**: Normal polarity PWM output
- **Channel D (CH4) - PA3**: Inverse polarity PWM output (complementary to CH3)

## Implementation Details

### Inverse Polarity Configuration

The key to achieving inverse polarity is setting the `LL_TIM_OCPOLARITY_LOW` for the inverse channels while keeping normal channels at `LL_TIM_OCPOLARITY_HIGH`.

#### Timer 1 Configuration
```c
/* Channel 1 (Normal polarity) */
LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);

/* Channel 2 (Inverse polarity) */
LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_LOW);
LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_HIGH);
```

#### Timer 2 Configuration
```c
/* Channel 3 (Normal polarity) */  
LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);

/* Channel 4 (Inverse polarity) */
LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
```

### Key Configuration Parameters

| Parameter | Normal Channel | Inverse Channel | Effect |
|-----------|----------------|-----------------|--------|
| Output Polarity | `LL_TIM_OCPOLARITY_HIGH` | `LL_TIM_OCPOLARITY_LOW` | Inverts the output signal |
| Idle State (TIM1) | `LL_TIM_OCIDLESTATE_LOW` | `LL_TIM_OCIDLESTATE_HIGH` | Complementary idle states |
| Compare Value | Same for both channels | Same for both channels | Maintains phase relationship |

## Signal Behavior

When configured correctly, the inverse polarity channels will be 180° out of phase with their corresponding normal channels.

## Hardware Connections

| Signal | STM32 Pin | Function | Polarity |
|--------|-----------|----------|----------|
| TIM1_CH1 | PA8 | Timer 1 Channel A | Normal |
| TIM1_CH2 | PA9 | Timer 1 Channel B | Inverse |
| TIM2_CH3 | PA2 | Timer 2 Channel C | Normal |
| TIM2_CH4 | PA3 | Timer 2 Channel D | Inverse |

## Build Instructions

```bash
# Compile the project (requires ARM GCC toolchain)
make

# For syntax checking without ARM toolchain
make check

# For simulation
make simulate
```

## Running Tests

```bash
cd tests
make run_tests
```


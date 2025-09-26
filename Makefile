# STM32F103 8-bit Clock Generator Makefile

# Target configuration
TARGET = stm32f103_clock_generator
MCU = STM32F103xB

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build

# Tools
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Compiler flags
CFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
CFLAGS += -DUSE_HAL_DRIVER -D$(MCU)
CFLAGS += -I$(INC_DIR)
CFLAGS += -Os -g3 -Wall -ffunction-sections -fdata-sections
CFLAGS += -std=c99

# Linker flags
LDFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $<

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

# Generate hex file
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# Generate binary file
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Show frequency mapping table
frequency_table:
	@echo "Frequency Mapping Table:"
	@echo "========================"
	@echo "ADC Range    | Frequency Range | Description"
	@echo "-------------|-----------------|------------------"
	@echo "0-819 (20%)  | 1Hz - 100Hz     | Low frequency range"
	@echo "820-4095     | 100Hz - 100kHz  | High frequency range"
	@echo ""
	@echo "Key points:"
	@echo "- ADC 0:    1Hz"
	@echo "- ADC 819:  100Hz"
	@echo "- ADC 820:  100Hz"
	@echo "- ADC 4095: 100kHz"

# Test frequency calculations (requires running on target or simulation)
test_frequencies:
	@echo "Test cases for frequency mapping:"
	@echo "================================="
	@echo "These values should be verified on target hardware"
	@echo ""
	@echo "ADC Value | Expected Frequency"
	@echo "----------|------------------"
	@echo "0         | 1 Hz"
	@echo "410       | ~50 Hz"
	@echo "819       | 100 Hz"
	@echo "820       | 100 Hz"
	@echo "2457      | ~50 kHz"
	@echo "4095      | 100 kHz"

.PHONY: all clean frequency_table test_frequencies
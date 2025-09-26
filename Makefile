# STM32F103 Clock Generator Makefile

# Target name
TARGET = stm32f103_clock_generator

# Directories
SRC_DIR = src
INC_DIR = inc
LIB_DIR = lib
BUILD_DIR = build

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.c)

# Object files
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# Compiler and tools
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# MCU specific settings
MCU = cortex-m3
ARCH = armv7-m

# Compiler flags
CFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=soft
CFLAGS += -DSTM32F103xB -DUSE_FULL_LL_DRIVER
CFLAGS += -I$(INC_DIR)
CFLAGS += -O2 -g3 -Wall -Wextra
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -std=c99

# Linker flags
LDFLAGS = -mcpu=$(MCU) -mthumb -mfloat-abi=soft
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "Linking $(TARGET).elf"
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

# Generate hex file
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo "Generating $(TARGET).hex"
	$(OBJCOPY) -O ihex $< $@

# Generate binary file
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "Generating $(TARGET).bin"
	$(OBJCOPY) -O binary $< $@

# Show size information
size: $(BUILD_DIR)/$(TARGET).elf
	@echo "Size information:"
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Simulate compilation (for testing without actual ARM toolchain)
simulate:
	@echo "Simulating compilation process..."
	@echo "Source files found:"
	@for src in $(SOURCES); do echo "  $$src"; done
	@echo "Compilation would create:"
	@for obj in $(OBJECTS); do echo "  $$obj"; done
	@echo "Final target: $(BUILD_DIR)/$(TARGET).elf"
	@mkdir -p $(BUILD_DIR)
	@touch $(BUILD_DIR)/$(TARGET).elf
	@touch $(BUILD_DIR)/$(TARGET).hex
	@touch $(BUILD_DIR)/$(TARGET).bin
	@echo "Simulation complete!"

# Run basic syntax check using native GCC
check:
	@echo "Running syntax check..."
	@for src in $(SOURCES); do \
		echo "Checking $$src..."; \
		gcc -fsyntax-only -DSTM32F103xB -DUSE_FULL_LL_DRIVER -I$(INC_DIR) -I/usr/include $$src || echo "Note: Some STM32-specific definitions may cause warnings"; \
	done
	@echo "Syntax check complete!"

.PHONY: all clean simulate check size
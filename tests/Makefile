# Test Makefile for STM32F103 Clock Generator

CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

# Test targets
TEST_SOURCES = test_functionality.c
TEST_TARGETS = $(TEST_SOURCES:.c=)

all: $(TEST_TARGETS)

test_functionality: test_functionality.c
	$(CC) $(CFLAGS) $< -o $@

run_tests: $(TEST_TARGETS)
	@echo "Running comprehensive functionality tests..."
	./test_functionality

clean:
	rm -f $(TEST_TARGETS)

.PHONY: all run_tests clean
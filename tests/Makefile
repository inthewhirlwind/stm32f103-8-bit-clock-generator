# Test Makefile for STM32F103 Clock Generator

CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

# Test targets
TEST_SOURCES = test_inverse_polarity.c
TEST_TARGETS = $(TEST_SOURCES:.c=)

all: $(TEST_TARGETS)

test_inverse_polarity: test_inverse_polarity.c
	$(CC) $(CFLAGS) $< -o $@

run_tests: $(TEST_TARGETS)
	@echo "Running inverse polarity tests..."
	./test_inverse_polarity

clean:
	rm -f $(TEST_TARGETS)

.PHONY: all run_tests clean
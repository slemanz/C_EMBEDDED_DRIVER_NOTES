# Toolchain
CC = gcc
CXX = g++
LD = g++

# Directories
SRC_DIR = src
TEST_DIR = tests
BUILD_DIR = build

SRCS = $(wildcard $(SRC_DIR)/*.c)
TEST_SRCS = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
TEST_OBJS = $(patsubst $(TEST_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(TEST_SRCS))

# Flags
CFLAGS = -Wall -Wextra -g -I$(SRC_DIR)
CXXFLAGS = $(CFLAGS)
LDFLAGS = -lCppUTest -lCppUTestExt

# Targets
.PHONY: all clean test

all: test

print_variable:
	@echo $(SRCS)
	@echo $(TEST_SRCS)
	@echo $(OBJS)
	@echo $(TEST_OBJS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(OBJS) $(TEST_OBJS)
	$(LD) -o $(BUILD_DIR)/run_tests $^ $(LDFLAGS)
	$(BUILD_DIR)/run_tests

clean:
	rm -rf $(BUILD_DIR)

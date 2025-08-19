CC= gcc
INCLUDES = -I unity -I src

SRC_DIR = src
TEST_DIR = tests
BUILD_DIR = build
UNITY_DIR = unity

SRCS = $(SRC_DIR)/math_utils.c
TESTS = $(TEST_DIR)/test_math_utils.c 
UNITY = $(UNITY_DIR)/unity.c

all: 

test: $(SRCS) $(TESTS) $(UNITY)
	mkdir -p $(BUILD_DIR)
	$(CC) $(INCLUDES) $^ -o $(BUILD_DIR)/run_tests
	./$(BUILD_DIR)/run_tests

clean:
	rm -rf $(BUILD_DIR)/*
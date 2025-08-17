#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include <cstring>

extern "C" {
    #include "../src/gpio.h"
}

// Test group for GPIO
TEST_GROUP(GPIO_Tests)
{
    GPIO_TypeDef testGPIO;

    void setup()
    {
        // Initialize all registers to 0 before each test
        memset(&testGPIO, 0, sizeof(GPIO_TypeDef));
    }

    void teardown()
    {
        mock().clear();
    }
};

TEST(GPIO_Tests, GPIO_Init_Output)
{
    GPIO_Init(&testGPIO, 5, GPIO_MODE_OUTPUT, GPIO_NOPULL);
    
    // Check MODER is set correctly (01 for output)
    UNSIGNED_LONGS_EQUAL(0x1 << (2 * 5), testGPIO.MODER);
    UNSIGNED_LONGS_EQUAL(0, testGPIO.PUPDR);
}

TEST(GPIO_Tests, GPIO_Init_Input_Pullup)
{
    GPIO_Init(&testGPIO, 3, GPIO_MODE_INPUT, GPIO_PULLUP);

    // check MODER is set correctly (00 for input)
    UNSIGNED_LONGS_EQUAL(0x0 << (2*3), testGPIO.MODER & (0x3 << (2*3)));

    // Check PUPDR is set correctly (01 for pull-up)
    UNSIGNED_LONGS_EQUAL(0x0 << (2*3), testGPIO.MODER & (0x3 << (2*3)));
}

// Test GPIO write operations
TEST(GPIO_Tests, GPIO_WritePin_Set)
{
    GPIO_WritePin(&testGPIO, 7, GPIO_PIN_SET);
    UNSIGNED_LONGS_EQUAL(1 << 7, testGPIO.BSRR);
}

TEST(GPIO_Tests, GPIO_WritePin_Reset)
{
    GPIO_WritePin(&testGPIO, 4, GPIO_PIN_RESET);
    UNSIGNED_LONGS_EQUAL(1 << (4+16), testGPIO.BSRR);
}

// Test GPIO read operations
TEST(GPIO_Tests, GPIO_ReadPin_High) {
    testGPIO.IDR = (1 << 2);
    LONGS_EQUAL(GPIO_PIN_SET, GPIO_ReadPin(&testGPIO, 2));
}

TEST(GPIO_Tests, GPIO_ReadPin_Low) {
    testGPIO.IDR = 0;
    LONGS_EQUAL(GPIO_PIN_RESET, GPIO_ReadPin(&testGPIO, 1));
}
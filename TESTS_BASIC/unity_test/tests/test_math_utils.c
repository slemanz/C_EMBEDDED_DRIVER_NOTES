#include "unity.h"
#include "math_utils.h"

void setUp(void)
{
    // optional - run before each test
}

void tearDown(void)
{
    // optional - run after each test
}

void test_add(void)
{
    TEST_ASSERT_EQUAL_INT(5, add(2, 3));
}

void test_add_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, add(0, 0));
}

void test_add_two_plus_two(void)
{
    TEST_ASSERT_EQUAL_INT(4, add(2, 2));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_add);
    RUN_TEST(test_add_zero);
    RUN_TEST(test_add_two_plus_two);

    return UNITY_END();
}

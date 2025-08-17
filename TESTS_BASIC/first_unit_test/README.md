# First Unit Test

Unit testing verifies individual units of source code work as intended. In embedded systems, this means:

- Testing hardware-independent logic
- Mocking hardware dependencies
- Running tests on host machine (faster development)

Benefits:

- Catch bugs early
- Enable refactoring with confidence
- Document expected behavior
- Reduce debugging time

### Setting Up the Environment 

Prerequisites:

- Toolchain (arm-none-eabi-gcc)
- Make
- CppUTest (we'll install it)

Install CppUTest:

```bash
# On Ubuntu/Debian
sudo apt-get install cpputest

# Or build from source
git clone https://github.com/cpputest/cpputest.git
cd cpputest
./autogen.sh
./configure
make
sudo make install
```

###  Project Structure

```
stm32_unit_test/
├── Makefile
├── src/
│   ├── gpio.c
│   └── gpio.h
├── tests/
│   ├── test_gpio.cpp
│   └── AllTests.cpp
└── stubs/
    └── stm32f4xx.h
```

### Creating a Simple GPIO Driver

Let's create a basic GPIO driver for our STM32F411.

- [src/gpio.h](src/gpio.h)
- [src/gpio.c](src/gpio.c)

### Writing Unit Tests with CppUTest

- [tests/AllTests.cpp](tests/AllTests.cpp)
- [tests/test_gpio.cpp](tests/test_gpio.cpp)
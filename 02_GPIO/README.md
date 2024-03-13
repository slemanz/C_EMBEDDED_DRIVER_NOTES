# GPIO DRIVER

The gpio driver API requirements:

* GPIO initialization

* Enable/disable GPIO port clock

* Read from GPIO pin/port 

* Write to GPIO pin/port

* Configure alternate functionality

* Interrupt Handling

## Files

The main files create to the GPIO driver:

* [STMF401xx header file](drivers/Inc/stm32f401xx.h)

* [Gpio header file](drivers/Inc/gpio.h)

* [Gpio source file](drivers/Src/gpio.c)

## Applications

1. [Toggle Led](Src/01led_toggle.c) - code to toggle a led with a for delay.

2. [Button Led](Src/01button_led.c) - code to toggle a led when a external button is pressed.


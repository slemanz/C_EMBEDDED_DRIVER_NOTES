# Great Commits Examples

1. **Feature Addition (feat):** New functionality in firmware/drivers.
    - drivers(feat): add SPI support for SD card module
    - rtos(feat): integrate FreeRTOS task scheduler
    - sensors(feat): implement BME280 temperature/pressure driver
    - wireless(feat): add LoRaWAN join mechanism
    - bootloader(feat): support OTA firmware updates
    - power(feat): implement low-power sleep mode
    - lcd(feat): add ST7789 TFT driver with DMA acceleration
    - usb(feat): implement CDC-ACM for serial debug console
    - security(feat): integrate AES-256 encryption for OTA updates
    - sensors(feat): add support for MAX30102 pulse oximeter

2. **Bug Fixes (fix):** Hardware/software workarounds or corrections.
    - i2c(fix): handle bus lockup on NACK
    - uart(fix): correct baud rate calculation for STM32F4
    - adc(fix): calibrate voltage reference drift
    - watchdog(fix): reset timeout for ESP32
    - motor(fix): eliminate PWM jitter at 100% duty cycle
    - ble(fix): resolve GATT service discovery hang
    - can(fix): handle bus-off recovery in CAN controller
    - rtc(fix): compensate for DS3231 temperature drift
    - flash(fix): prevent write corruption during power loss
    - pwm(fix): correct dead-time calculation for H-bridge

3. **Documentation (docs):** Hardware-specific notes or drivers.
    - hw(docs): add pinout diagram for custom PCB
    - protocol(docs): describe CAN bus message format
    - firmware(docs): update FLASH memory map
    - api(docs): document driver HAL functions
    - safety(docs): add ESD handling guidelines
    - toolchain(docs): setup guide for ARM GCC
    - schematic(docs): annotate power tree with max current ratings
    - registers(docs): add bitfield map for IMU config register
    - api(docs): document thread-safety rules for HAL functions

4. **Refactoring (refactor):** Code structure improvements.
    - hal(refactor): decouple STM32 HAL from application logic
    - drivers(refactor): generalize SPI interface for multiple ICs
    - isr(refactor): optimize interrupt priority grouping
    - memory(refactor): dynamic allocation pool for RTOS
    - cmake(refactor): modularize firmware build system
    - cli(refactor): migrate from UART to USB-CDC
    - hal(refactor): replace vendor HAL with register-based drivers
    - isr(refactor): group GPIO interrupts by priority bank
    - memory(refactor): move stack to external SRAM for RTOS tasks
    - build(refactor): split monolithic firmware into modular libraries

5. **Performance (perf):** Speed/memory optimizations.
    - dma(perf): accelerate ADC sampling via circular buffer
    - crypto(perf): replace software AES with HW acceleration
    - motor(perf): reduce step latency from 100µs → 10µs
    - memory(perf): optimize SRAM usage by 15%
    - wireless(perf): reduce LoRa TX current by 20mA
    - isr(perf): shrink ISR latency by disabling unused peripherals
    - adc(perf): oversample to 14-bit resolution in software
    - motor(perf): implement sensorless field-oriented control (FOC)
    - radio(perf): reduce LoRa preamble time for faster TX
    - sleep(perf): drop STM32 stop mode current to 2µA

6. **Tests (tests):** Hardware validation/QA.
    - eeprom(tests): validate 100k write cycles
    - rtos(tests): stress test task switching
    - i2c(tests): check multi-master arbitration
    - power(tests): measure µA sleep current
    - fuzz(tests): inject garbage UART data
    - hw(tests): automated probe continuity check
    - emc(tests): validate immunity to 15kV ESD strikes
    - stress(tests): run 72-hour motor duty cycle test
    - boundary(tests): check ADC at min/max VREF voltages
    - fuzz(tests): flood UART with malformed packets

7. **Chores (chore):** Maintenance/tooling updates.
    - toolchain(chore): upgrade to GCC 12.2
    - ci(chore): add Renode emulator to pipeline
    - deps(chore): update ESP-IDF to v5.1
    - git(chore): add .gitattributes for LF/CRLF
    - hw(chore): rebase PCB design on KiCad 7
    - make(chore): auto-detect connected debugger
    - ci(chore): add HIL (Hardware-in-Loop) test automation
    - deps(chore): bump CMSIS to v5.9.0
    - git(chore): add .gitignore for Keil IDE artifacts
    - hw(chore): migrate PCB design to Altium 24

8. **Breaking Changes (break!):** Backward-incompatible updates.
    - hal(break)!: migrate from HAL to LL drivers
    - api(break)!: require callback registration for BLE events
    - memory(break)!: relocate heap to external SRAM
    - build(break)!: require CMake 3.20+
    - protocol(break)!: switch from Modbus to CANopen
    - power(break)!: remove 3.3V rail support
    - ci(chore): add HIL (Hardware-in-Loop) test automation
    - deps(chore): bump CMSIS to v5.9.0
    - git(chore): add .gitignore for Keil IDE artifacts
    - toolchain(break)!: require IAR Embedded Workbench 9.30+

9. **Revert (revert):** Rollbacks in firmware.
    - bootloader(revert): undo faulty OTA patch (brick risk)
    - sensors(revert): roll back BME280 driver (I2C glitches)
    - rtos(revert): remove priority inheritance (deadlocks)
    - dma(revert): disable double-buffering (corruption)
    - wireless(revert): fallback to ESP-AT commands (stability)
    - isr(revert): restore old NVIC config (timing issues)
    - dma(revert): remove double-buffered ADC (data corruption)
    - bootloader(revert): fall back to UART DFU (USB unstable)
    - safety(revert): restore watchdog timeout to 1s (false triggers)

10. **Style (style):** Firmware code formatting.
    - linter(style): enforce MISRA C guidelines
    - clang(style): apply .clang-format to drivers/
    - comments(style): add Doxygen to all ISRs
    - naming(style): rename GPIO_Pin→LED_PIN globally
    - indent(style): convert tabs to 4 spaces
    - header(style): standardize license preamble
    - misra(style): fix Rule 8.4 (declaration alignment)
    - comments(style): add ISR pre/post conditions in Doxygen
    - indent(style): convert all switch cases to braces
# Library Description

**Author:** Ehsan Zehni

**Short Description for GitHub:**

Complete embedded library for HLW8032 energy metering IC with UART communication, Timer Input Capture for PF measurement, calibration, filtering, statistics, and callback support. Production-ready for STM32 and other microcontrollers.

**Detailed Description:**

A comprehensive, production-ready C library for interfacing with HLW8032 single-phase energy metering IC. The library provides complete UART frame handling (24-byte frames), Timer Input Capture support for PF pin frequency measurement, automatic power factor calculation, calibration capabilities, moving average filtering, statistics collection, and callback functions for asynchronous processing.

**Key Features:**
- Full UART frame reception and parsing with CRC validation
- Timer Input Capture for accurate PF frequency measurement
- Automatic power factor calculation
- Calibration support (voltage, current, power, PF)
- Moving average filter for noise reduction
- Complete statistics (max, min, average, error counts)
- Callback functions for frame and error events
- Advanced calculations (apparent power, reactive power, energy cost)
- Modular architecture for easy portability

**Target Platforms:**
- STM32 (HAL)
- Other ARM Cortex-M microcontrollers
- Any platform with UART and Timer support

**Use Cases:**
- Energy monitoring systems
- Smart meters
- Energy management systems
- Energy cost calculation
- Power quality analysis
- IoT energy applications
- Educational projects


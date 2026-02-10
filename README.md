# TMP102 Temperature Sensor ESP-IDF Component

C API for TI TMP102 I2C temperature sensor using board_hal abstraction.

## Overview

Supports TMP102 12-bit/13-bit digital temperature sensor with I2C interface.

## Features

- Read temperature in Celsius or Fahrenheit
- Sleep/wakeup modes for power management
- Configurable temperature alerts with thresholds
- One-shot conversion mode
- Extended mode support (-55°C to +150°C)

## API Summary

- `tmp102_init()` - Initialize sensor
- `tmp102_read_temp_c()` / `tmp102_read_temp_f()` - Read temperature
- `tmp102_sleep()` / `tmp102_wakeup()` - Power management
- `tmp102_set_*()` - Configure sensor settings
- See `include/tmp102.h` for full API documentation

## Hardware Support

I2C addresses: 0x48 (GND), 0x49 (VCC), 0x4A (SDA), 0x4B (SCL)

## Dependencies

Requires `board_hal` component for I2C communication.

## License

Derived from SparkFun TMP102 Library (beerware license).

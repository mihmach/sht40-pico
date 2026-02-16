# Pico SHT4x (SHT40) driver (pico-sdk + PlatformIO)

This project contains a small SHT4x (SHT40/SHT41/SHT45) I2C driver adapted for RP2040 / Pico using **pico-sdk**.

## Wiring

- SHT4x `VDD` -> Pico `3V3`
- SHT4x `GND` -> Pico `GND`
- SHT4x `SDA` -> Pico `GPIO4` (I2C0 SDA by default)
- SHT4x `SCL` -> Pico `GPIO5` (I2C0 SCL by default)

Use pull-ups to 3.3V (many breakout boards already have them).

## Library

- `lib/sht4x/include/SHT4x.h`
- `lib/sht4x/src/SHT4x.cpp`

Example usage is in `src/main.cpp`.


#pragma once

#include <cstdint>
#include <cstddef>

#include "hardware/i2c.h"

class SHT4x {
public:
  enum class Precision : uint8_t {
    High,
    Medium,
    Low,
  };

  enum class Heater : uint8_t {
    Off,
    High_1s,
    High_100ms,
    Medium_1s,
    Medium_100ms,
    Low_1s,
    Low_100ms,
  };

  enum class Status : int {
    Ok = 0,
    InvalidArg = -1,
    I2cWriteFailed = -2,
    I2cReadFailed = -3,
    CrcMismatch = -4,
  };

  explicit SHT4x(uint8_t address = 0x44);

  void setBus(i2c_inst_t* i2c);
  uint8_t address() const;

  Status softReset();
  Status readSerial(uint32_t& serial);

  Status read(float& temperature_c, float& humidity_rh, Precision precision = Precision::High);
  Status readWithHeater(float& temperature_c, float& humidity_rh, Heater heater);

  static float rawToTemperatureC(uint16_t raw);
  static float rawToHumidityRH(uint16_t raw);

private:
  i2c_inst_t* i2c_ = nullptr;
  uint8_t address_ = 0x44;

  static constexpr uint8_t kCmdSoftReset = 0x94;
  static constexpr uint8_t kCmdReadSerial = 0x89;

  static constexpr uint8_t kCmdMeasureHigh = 0xFD;
  static constexpr uint8_t kCmdMeasureMedium = 0xF6;
  static constexpr uint8_t kCmdMeasureLow = 0xE0;

  static constexpr uint8_t kCmdHeaterHigh1s = 0x39;
  static constexpr uint8_t kCmdHeaterHigh100ms = 0x32;
  static constexpr uint8_t kCmdHeaterMedium1s = 0x2F;
  static constexpr uint8_t kCmdHeaterMedium100ms = 0x24;
  static constexpr uint8_t kCmdHeaterLow1s = 0x1E;
  static constexpr uint8_t kCmdHeaterLow100ms = 0x15;

  Status writeCommand(uint8_t command);
  Status readWordsWithCrc(uint16_t* words, size_t word_count);

  static uint8_t crc8(const uint8_t* data, size_t len);
  static bool checkWordCrc(uint16_t word, uint8_t crc);
};


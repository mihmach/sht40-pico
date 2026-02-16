#include "SHT4x.h"

#include "pico/stdlib.h"

namespace {
static inline int sleep_for_command(uint8_t command) {
  // Datasheet typical max times are small; these conservative delays avoid busy-looping.
  switch (command) {
    case 0xFD: return 10; // measure high precision
    case 0xF6: return 5;  // measure medium precision
    case 0xE0: return 2;  // measure low precision

    case 0x89: return 1;  // read serial
    case 0x94: return 1;  // soft reset

    case 0x39: return 1100; // heater high 1s
    case 0x2F: return 1100; // heater medium 1s
    case 0x1E: return 1100; // heater low 1s

    case 0x32: return 110; // heater high 100ms
    case 0x24: return 110; // heater medium 100ms
    case 0x15: return 110; // heater low 100ms
    default: return 1;
  }
}
} // namespace

SHT4x::SHT4x(uint8_t address) : address_(address) {}

void SHT4x::setBus(i2c_inst_t* i2c) { i2c_ = i2c; }

uint8_t SHT4x::address() const { return address_; }

SHT4x::Status SHT4x::softReset() {
  if (i2c_ == nullptr) return Status::InvalidArg;
  Status st = writeCommand(kCmdSoftReset);
  if (st != Status::Ok) return st;
  sleep_ms(sleep_for_command(kCmdSoftReset));
  return Status::Ok;
}

SHT4x::Status SHT4x::readSerial(uint32_t& serial) {
  if (i2c_ == nullptr) return Status::InvalidArg;
  Status st = writeCommand(kCmdReadSerial);
  if (st != Status::Ok) return st;
  sleep_ms(sleep_for_command(kCmdReadSerial));

  uint16_t words[2] = {};
  st = readWordsWithCrc(words, 2);
  if (st != Status::Ok) return st;

  serial = (static_cast<uint32_t>(words[0]) << 16) | static_cast<uint32_t>(words[1]);
  return Status::Ok;
}

SHT4x::Status SHT4x::read(float& temperature_c, float& humidity_rh, Precision precision) {
  if (i2c_ == nullptr) return Status::InvalidArg;

  uint8_t cmd = kCmdMeasureHigh;
  switch (precision) {
    case Precision::High: cmd = kCmdMeasureHigh; break;
    case Precision::Medium: cmd = kCmdMeasureMedium; break;
    case Precision::Low: cmd = kCmdMeasureLow; break;
    default: return Status::InvalidArg;
  }

  Status st = writeCommand(cmd);
  if (st != Status::Ok) return st;
  sleep_ms(sleep_for_command(cmd));

  uint16_t words[2] = {};
  st = readWordsWithCrc(words, 2);
  if (st != Status::Ok) return st;

  temperature_c = rawToTemperatureC(words[0]);
  humidity_rh = rawToHumidityRH(words[1]);
  return Status::Ok;
}

SHT4x::Status SHT4x::readWithHeater(float& temperature_c, float& humidity_rh, Heater heater) {
  if (i2c_ == nullptr) return Status::InvalidArg;
  if (heater == Heater::Off) return read(temperature_c, humidity_rh, Precision::High);

  uint8_t cmd = 0;
  switch (heater) {
    case Heater::High_1s: cmd = kCmdHeaterHigh1s; break;
    case Heater::High_100ms: cmd = kCmdHeaterHigh100ms; break;
    case Heater::Medium_1s: cmd = kCmdHeaterMedium1s; break;
    case Heater::Medium_100ms: cmd = kCmdHeaterMedium100ms; break;
    case Heater::Low_1s: cmd = kCmdHeaterLow1s; break;
    case Heater::Low_100ms: cmd = kCmdHeaterLow100ms; break;
    default: return Status::InvalidArg;
  }

  Status st = writeCommand(cmd);
  if (st != Status::Ok) return st;
  sleep_ms(sleep_for_command(cmd));

  uint16_t words[2] = {};
  st = readWordsWithCrc(words, 2);
  if (st != Status::Ok) return st;

  temperature_c = rawToTemperatureC(words[0]);
  humidity_rh = rawToHumidityRH(words[1]);
  return Status::Ok;
}

float SHT4x::rawToTemperatureC(uint16_t raw) {
  // T = -45 + 175 * raw / 65535
  return -45.0f + (175.0f * static_cast<float>(raw) / 65535.0f);
}

float SHT4x::rawToHumidityRH(uint16_t raw) {
  // RH = -6 + 125 * raw / 65535, clamp 0..100
  float rh = -6.0f + (125.0f * static_cast<float>(raw) / 65535.0f);
  if (rh < 0.0f) rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  return rh;
}

SHT4x::Status SHT4x::writeCommand(uint8_t command) {
  uint8_t buf[1] = {command};

  // pico-sdk i2c_write_blocking returns number of bytes written, or PICO_ERROR_GENERIC (-1).
  int written = i2c_write_blocking(i2c_, address_, buf, 1, false);
  if (written != 1) return Status::I2cWriteFailed;
  return Status::Ok;
}

SHT4x::Status SHT4x::readWordsWithCrc(uint16_t* words, size_t word_count) {
  if (word_count == 0) return Status::Ok;

  // Each word is: MSB LSB CRC
  const size_t bytes_to_read = word_count * 3;
  uint8_t rx[3 * 2] = {}; // currently used for 2 words; keep simple.
  if (bytes_to_read > sizeof(rx)) return Status::InvalidArg;

  int read = i2c_read_blocking(i2c_, address_, rx, static_cast<int>(bytes_to_read), false);
  if (read != static_cast<int>(bytes_to_read)) return Status::I2cReadFailed;

  for (size_t i = 0; i < word_count; ++i) {
    const uint8_t msb = rx[i * 3 + 0];
    const uint8_t lsb = rx[i * 3 + 1];
    const uint8_t crc = rx[i * 3 + 2];
    const uint16_t word = (static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb);
    if (!checkWordCrc(word, crc)) return Status::CrcMismatch;
    words[i] = word;
  }

  return Status::Ok;
}

uint8_t SHT4x::crc8(const uint8_t* data, size_t len) {
  // Sensirion CRC-8: polynomial 0x31 (x^8 + x^5 + x^4 + 1), init 0xFF.
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool SHT4x::checkWordCrc(uint16_t word, uint8_t crc) {
  uint8_t buf[2] = {static_cast<uint8_t>(word >> 8), static_cast<uint8_t>(word & 0xFF)};
  return crc8(buf, 2) == crc;
}

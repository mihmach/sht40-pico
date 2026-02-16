#include <cstdio>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "SHT4x.h"

namespace {
constexpr i2c_inst_t* kI2c = i2c0;
constexpr uint kI2cSdaPin = 4;
constexpr uint kI2cSclPin = 5;
constexpr uint32_t kI2cBaudrate = 400'000;
} // namespace

int main() {
  stdio_init_all();

  i2c_init(kI2c, kI2cBaudrate);
  gpio_set_function(kI2cSdaPin, GPIO_FUNC_I2C);
  gpio_set_function(kI2cSclPin, GPIO_FUNC_I2C);
  gpio_pull_up(kI2cSdaPin);
  gpio_pull_up(kI2cSclPin);

  SHT4x sht;
  sht.setBus(kI2c);

  (void)sht.softReset();

  while (true) {
    float t_c = 0.0f;
    float rh = 0.0f;
    const auto st = sht.read(t_c, rh, SHT4x::Precision::High);
    if (st == SHT4x::Status::Ok) {
      std::printf("T=%.2fC RH=%.2f%%\n", static_cast<double>(t_c), static_cast<double>(rh));
    } else {
      std::printf("SHT4x read failed: %d\n", static_cast<int>(st));
    }
    sleep_ms(1000);
  }
}


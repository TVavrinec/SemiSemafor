#pragma once

#include <driver/gpio.h>

namespace Pins {

static constexpr gpio_num_t SW1 = GPIO_NUM_0;
static constexpr gpio_num_t SW2 = GPIO_NUM_1;
static constexpr gpio_num_t Leds = GPIO_NUM_8;

static constexpr int LED_COUNT = 12;

namespace IMU::SPI {

static constexpr gpio_num_t SCK = GPIO_NUM_4;
static constexpr gpio_num_t CS = GPIO_NUM_5;
static constexpr gpio_num_t SDO = GPIO_NUM_6;
static constexpr gpio_num_t SDI = GPIO_NUM_7;

} // namespace IMU::SPI

namespace IMU::I2C {

static constexpr gpio_num_t SDA = GPIO_NUM_7;
static constexpr gpio_num_t SCL = GPIO_NUM_4;

} // namespace IMU::I2C

namespace IMU {

static constexpr gpio_num_t INT = GPIO_NUM_10;

} // namespace IMU

} // namespace Pins

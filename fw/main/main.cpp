#include "IMU.hpp"
#include "pinout.hpp"

#include "SmartLeds.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <thread>
#include <cstdint>
#include <cstdio>

static constexpr const char* TAG = "main";
using namespace std::chrono_literals;

void initPins()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << Pins::SW1) | (1ULL << Pins::SW2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

bool readSwitch(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

std::tuple<int, int, int> calcDiff(std::tuple<int, int, int> newValue) {
    static std::tuple<int, int, int> oldValue = {0, 0, 0};
    std::tuple<int, int, int> diff = {0, 0, 0};
    diff = {std::get<0>(newValue) - std::get<0>(oldValue), std::get<1>(newValue) - std::get<1>(oldValue), std::get<2>(newValue) - std::get<2>(oldValue)};
    oldValue = newValue;
    return diff;
}

extern "C" void app_main(void)
{
    initPins();

    SmartLed leds(LED_WS2812B, 12, Pins::Leds, 0, SingleBuffer);
    using namespace Pins::IMU;
    IMU imu(SPI::SDO, SPI::SDI, SPI::SCK, SPI::CS);
    imu.updateByte(IMU::CTRL_REG1, 0b111 << 4, 0b111 << 4); // Set sampling rate
    imu.updateByte(IMU::CTRL_REG2, 0b1 << 2, 0b1 << 2); // Enable click high-pass filter
    imu.readByte(IMU::REFERENCE); // This is probably needed per datasheet (to reset highpass filter), but who knows
    imu.updateByte(IMU::CTRL_REG3, 1 << 7, 1 << 7); // Enable click interrupt
    imu.writeByte(IMU::CLICK_CFG, 1 << 4); // Enable single click interrupt on z-axis
    imu.writeByte(IMU::CLICK_SRC, 1 << 4); // Enable single click detection
    imu.writeByte(IMU::CLICK_THS, 1 << 7 | 0x3F); // Enable latch

    while (true) {
        if (readSwitch(Pins::SW1)) {
            ESP_LOGI(TAG, "SW1 pressed");
            for (auto& led : leds)
                led = Rgb { 10, 0, 0 };
        } else if (readSwitch(Pins::SW2)) {
            ESP_LOGI(TAG, "SW2 pressed");
            for (auto& led : leds)
                led = Rgb { 0, 10, 0 };
        } else {
            for (auto& led : leds)
                led = Rgb { 0, 0, 10 };
        }

        // auto [x, y, z] = calcDiff(imu.accel());
        // ESP_LOGI(TAG, "x: %d\ty: %d\tz: %d\t", x, y, z);

        auto clickRes = imu.readByte(IMU::CLICK_SRC);
        // ESP_LOGI(TAG, "Click register: %d", clickRes);
        if (clickRes & (1 << 2)) {
            ESP_LOGI(TAG, "Click detected");
            for (auto& led : leds)
                led = Rgb { 100, 100, 100 };
            imu.writeByte(IMU::CLICK_SRC, ~(1 << 7 | 1 << 4));
            std::this_thread::sleep_for(100ms);
        }
        // const char header[1] = {0x80};
        // std::int16_t data[3] = {x,y,z}
        // write(fileno(stdout), header, 1);
        // write(fileno(stdout), );

        leds.show();
        std::this_thread::sleep_for(10ms);
    }
}

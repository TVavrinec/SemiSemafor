#include "pinout.hpp"

#include "SmartLeds.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr const char* TAG = "main";

void initPins()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SW1) | (1ULL << SW2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

bool readSwitch(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

extern "C" void app_main(void)
{
    initPins();

    SmartLed leds(LED_WS2812B, 5, Leds, 0, SingleBuffer);

    while (true) {
        if (readSwitch(SW1)) {
            ESP_LOGI(TAG, "SW1 pressed");
            for (auto& led : leds)
                led = Rgb { 255, 0, 0 };
        } else if (readSwitch(SW2)) {
            ESP_LOGI(TAG, "SW2 pressed");
            for (auto& led : leds)
                led = Rgb { 0, 255, 0 };
        } else {
            for (auto& led : leds)
                led = Rgb { 0, 0, 255 };
        }
        leds.show();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
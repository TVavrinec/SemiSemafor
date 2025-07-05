#include "pinout.hpp"           // Must define SW1, SW2, Leds, LED_WS2812B, etc.
#include "SmartLeds.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_random.h>         // For generating random numbers

static constexpr const char* TAG = "main";

// Wi‑Fi AP configuration
const char* EXAMPLE_SSID = "SSID";
const char* EXAMPLE_PASS = "Password";
const int EXAMPLE_WIFI_CHANNEL = 1;
const int EXAMPLE_MAX_STA_CONN = 4;

//
// Initialize the button input pins
//
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

//
// Read the state of a given switch (returns true if pressed)
//
bool readSwitch(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

//
// Wi‑Fi event handler for AP client connect/disconnect events
//
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Device joined, MAC:" MACSTR ", AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Device left, MAC:" MACSTR ", AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

//
// Initialize the Wi‑Fi softAP
//
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {};
    wifi_config.ap.ssid_len = strlen(EXAMPLE_SSID);
    wifi_config.ap.channel = EXAMPLE_WIFI_CHANNEL;
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    strncpy((char*) wifi_config.ap.ssid, EXAMPLE_SSID, sizeof(wifi_config.ap.ssid));
    if (strlen(EXAMPLE_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        strncpy((char*) wifi_config.ap.password, EXAMPLE_PASS, sizeof(wifi_config.ap.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started. SSID: %s, password: %s, channel: %d",
             EXAMPLE_SSID, EXAMPLE_PASS, EXAMPLE_WIFI_CHANNEL);
}

//
// Main application entry point
//
extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi‑Fi softAP
    wifi_init_softap();

    // Initialize the input pins and LED array
    initPins();
    SmartLed leds(LED_WS2812B, 12, Leds, 0, SingleBuffer);

    // Main loop
    while (true) {
        // Check if either button is pressed
        if (readSwitch(SW1) || readSwitch(SW2)) {
            ESP_LOGI(TAG, "Button pressed – generating random red LED pattern");
            // While the button remains pressed, update the LED pattern continuously.
            while (readSwitch(SW1) || readSwitch(SW2)) {
                // Generate a random number between 1 and the total LED count (here, 5)
                int randomCount = (esp_random() % 7) + 1;
                // Light the first 'randomCount' LEDs red; turn the rest off.
                for (int i = 0; i < 12; i++) {
                    if (i < randomCount) {
                        leds[i] = Rgb { 255, 0, 0 };
                    } else {
                        leds[i] = Rgb { 0, 0, 0 };
                    }
                }
                leds.show();
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

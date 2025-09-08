#include "pinout.hpp"           // Must define SW1, SW2, Leds, LED_WS2812B, etc.
#include "SmartLeds.h"
#include <string.h>
#include <algorithm>
#include <array>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_timer.h>

static constexpr const char* TAG = "main";

/* ───────────────────── Wi-Fi AP configuration (UNCHANGED) ────────────────── */
const char* EXAMPLE_SSID          = "SSID";
const char* EXAMPLE_PASS          = "Password";
const int   EXAMPLE_WIFI_CHANNEL  = 1;
const int   EXAMPLE_MAX_STA_CONN  = 4;

/* ───────────────────────── GPIO helpers ──────────────────────────────────── */
void initPins()
{
    gpio_config_t io_conf{};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SW1) | (1ULL << SW2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

bool readSwitch(gpio_num_t pin)   // true when pressed (active-low wiring)
{
    return gpio_get_level(pin) == 0;
}

/* ───────────────────── Wi-Fi helpers (UNCHANGED) ─────────────────────────── */
static void wifi_event_handler(void*,
                               esp_event_base_t,
                               int32_t event_id,
                               void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        auto* event = (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI(TAG, "Device joined, MAC:" MACSTR ", AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(TAG, "Device left, MAC:" MACSTR ", AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

void wifi_init_softap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config{};
    wifi_config.ap.ssid_len       = strlen(EXAMPLE_SSID);
    wifi_config.ap.channel        = EXAMPLE_WIFI_CHANNEL;
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;

    strncpy((char*)wifi_config.ap.ssid, EXAMPLE_SSID,
            sizeof(wifi_config.ap.ssid));

    if (strlen(EXAMPLE_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        strncpy((char*)wifi_config.ap.password, EXAMPLE_PASS,
                sizeof(wifi_config.ap.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started. SSID: %s, password: %s, channel: %d",
             EXAMPLE_SSID, EXAMPLE_PASS, EXAMPLE_WIFI_CHANNEL);
}

/* ─────────────────────────── King of the Hill ───────────────────────────── */

static constexpr int NUM_LEDS            = 12;

/* Displayed teams (A/B/C) */
static constexpr int NUM_DISPLAY_TEAMS   = 3;
static constexpr int TEAM_A              = 0;
static constexpr int TEAM_B              = 1;
static constexpr int TEAM_C              = 2;

/* Holder states for cycling: includes a DUMMY that doesn't accrue time
   and is NOT displayed as its own segment. */
enum Holder : int { HOLDER_DUMMY = 0, HOLDER_A = 1, HOLDER_B = 2, HOLDER_C = 3 };
static constexpr int NUM_HOLDERS = 4;  // dummy + 3 teams

static constexpr uint8_t V_INACTIVE = 5;   // non-active brightness
static constexpr uint8_t V_ACTIVE   = 20;  // active holder brightness

// Team colours (HSV) — no white anywhere
static constexpr uint8_t TEAM_HUE [NUM_DISPLAY_TEAMS] = { 0, 85, 170 }; // R,G,B-ish
static constexpr uint8_t TEAM_SAT [NUM_DISPLAY_TEAMS] = { 255, 255, 255 };

inline Hsv teamColour(int teamIndex, bool isActive)
{
    uint8_t v = isActive ? V_ACTIVE : V_INACTIVE;
    return Hsv{ TEAM_HUE[teamIndex], TEAM_SAT[teamIndex], v };
}

/* Render only the 3 real teams in fixed order (A, B, C).
   If the current holder is dummy, no segment is highlighted. */
inline void renderRing3(SmartLed& leds,
                        const int counts[NUM_DISPLAY_TEAMS],
                        Holder holder)
{
    int activeDisplay = -1;
    if (holder == HOLDER_A) activeDisplay = TEAM_A;
    else if (holder == HOLDER_B) activeDisplay = TEAM_B;
    else if (holder == HOLDER_C) activeDisplay = TEAM_C;
    // else (dummy): activeDisplay remains -1 (no highlight)

    int cursor = 0;
    for (int t = 0; t < NUM_DISPLAY_TEAMS; ++t) {
        Hsv c = teamColour(t, t == activeDisplay);
        for (int i = 0; i < counts[t] && cursor < NUM_LEDS; ++i) {
            leds[cursor++] = c;
        }
    }
    // Safety fill (shouldn't happen if counts sum to NUM_LEDS)
    for (; cursor < NUM_LEDS; ++cursor) {
        leds[cursor] = teamColour(TEAM_A, false);
    }
    leds.show();
    leds.wait();
}

// Sort indices of the 3 teams by descending time (stable for ties).
static inline void sortByTimeDesc3(const uint64_t times[NUM_DISPLAY_TEAMS],
                                   int idx[NUM_DISPLAY_TEAMS])
{
    for (int i = 0; i < NUM_DISPLAY_TEAMS; ++i) idx[i] = i;
    for (int i = 0; i < NUM_DISPLAY_TEAMS - 1; ++i) {
        for (int j = i + 1; j < NUM_DISPLAY_TEAMS; ++j) {
            if ( (times[idx[j]] > times[idx[i]]) ||
                 (times[idx[j]] == times[idx[i]] && idx[j] < idx[i]) )
            {
                std::swap(idx[i], idx[j]);
            }
        }
    }
}

/* Enforce: if times differ, LED counts differ (when possible with ≥1 LED floor). */
static inline void enforceStrictOrdering3(const uint64_t times[NUM_DISPLAY_TEAMS],
                                          int ledsCnt[NUM_DISPLAY_TEAMS])
{
    int order[NUM_DISPLAY_TEAMS];
    sortByTimeDesc3(times, order);

    for (int pass = 0; pass < 6; ++pass) {
        bool changed = false;
        for (int r = 0; r < NUM_DISPLAY_TEAMS - 1; ++r) {
            int hi = order[r];
            int lo = order[r + 1];

            if (times[hi] == times[lo]) continue;        // equal times may tie
            if (ledsCnt[hi] > ledsCnt[lo]) continue;     // already strict

            // Borrow 1 from a lower-ranked team with >1 LED
            bool fixed = false;
            for (int k = NUM_DISPLAY_TEAMS - 1; k >= r + 1; --k) {
                int donor = order[k];
                if (donor == hi) continue;
                if (ledsCnt[donor] > 1) {
                    ledsCnt[donor]--;
                    ledsCnt[hi]++;
                    fixed = true;
                    changed = true;
                    break;
                }
            }
            // As a last resort, borrow from the immediate next if it has >1
            if (!fixed && ledsCnt[lo] > 1) {
                ledsCnt[lo]--;
                ledsCnt[hi]++;
                changed = true;
            }
        }
        if (!changed) break;
    }
}

/* Apportion LEDs among the 3 teams with:
   - at least 1 LED per team
   - proportional split of the remaining LEDs by cumulative time
   - enforce unequal-times ⇒ unequal-LEDs when feasible */
static inline void apportionCounts3(const uint64_t times[NUM_DISPLAY_TEAMS],
                                    int outCounts[NUM_DISPLAY_TEAMS])
{
    for (int i = 0; i < NUM_DISPLAY_TEAMS; ++i) outCounts[i] = 1;
    int remaining = NUM_LEDS - NUM_DISPLAY_TEAMS;   // 12 - 3 = 9
    if (remaining <= 0) return;

    uint64_t total = times[0] + times[1] + times[2];

    if (total == 0) {
        // Initial even split: 4-4-4
        int base = remaining / NUM_DISPLAY_TEAMS;   // 3
        int rem  = remaining % NUM_DISPLAY_TEAMS;   // 0
        for (int i = 0; i < NUM_DISPLAY_TEAMS; ++i) outCounts[i] += base;
        for (int i = 0; i < rem; ++i) outCounts[i]++;
        return;
    }

    // Largest remainders on the remaining LEDs
    double exact[NUM_DISPLAY_TEAMS];
    double frac [NUM_DISPLAY_TEAMS];
    int    extra[NUM_DISPLAY_TEAMS];
    int    extrasAssigned = 0;

    for (int i = 0; i < NUM_DISPLAY_TEAMS; ++i) {
        exact[i] = (double)times[i] * (double)remaining / (double)total;
        extra[i] = (int)exact[i];
        frac[i]  = exact[i] - extra[i];
        outCounts[i] += extra[i];
        extrasAssigned += extra[i];
    }

    int left = remaining - extrasAssigned;
    while (left > 0) {
        int best = 0;
        for (int i = 1; i < NUM_DISPLAY_TEAMS; ++i) {
            if ( (frac[i] > frac[best]) ||
                 (frac[i] == frac[best] && times[i] > times[best]) ||
                 (frac[i] == frac[best] && times[i] == times[best] && i < best) )
            {
                best = i;
            }
        }
        outCounts[best]++;
        frac[best] = -1.0;
        --left;
    }

    enforceStrictOrdering3(times, outCounts);
}

/* ───────────────────────────── app_main ─────────────────────────────────── */
extern "C" void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Wi-Fi softAP (unchanged) */
    wifi_init_softap();

    /* I/O & LEDs */
    initPins();
    SmartLed leds(LED_WS2812B, NUM_LEDS, Leds, 0, SingleBuffer);

    // Cumulative hold time (µs) for A/B/C (dummy is not tracked/displayed)
    uint64_t hold_us[NUM_DISPLAY_TEAMS] = {0, 0, 0};

    // Start with dummy holding; cycle includes dummy
    Holder holder = HOLDER_DUMMY;

    // Buttons (edge detection)
    bool prevSW1 = false;
    bool prevSW2 = false;

    // Initial draw: EVEN 4-4-4 split, no highlight (dummy)
    int counts[NUM_DISPLAY_TEAMS] = {4, 4, 4};
    renderRing3(leds, counts, holder);

    // Timekeeping
    uint64_t last_us = (uint64_t)esp_timer_get_time();

    while (true) {
        bool sw1 = readSwitch(SW1);   // NEXT
        bool sw2 = readSwitch(SW2);   // PREVIOUS

        // Cycle holder: Dummy <-> A <-> B <-> C
        if (prevSW1 && !sw1) {
            holder = static_cast<Holder>((holder + 1) % NUM_HOLDERS);
            ESP_LOGI(TAG, "SW1 released – holder -> %d", (int)holder);
        }
        if (prevSW2 && !sw2) {
            holder = static_cast<Holder>((holder + NUM_HOLDERS - 1) % NUM_HOLDERS);
            ESP_LOGI(TAG, "SW2 released – holder -> %d", (int)holder);
        }
        prevSW1 = sw1;
        prevSW2 = sw2;

        // Accumulate time for real teams only (ignore dummy)
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        uint64_t dt_us  = now_us - last_us;
        last_us = now_us;

        if (holder == HOLDER_A)      hold_us[TEAM_A] += dt_us;
        else if (holder == HOLDER_B) hold_us[TEAM_B] += dt_us;
        else if (holder == HOLDER_C) hold_us[TEAM_C] += dt_us;
        // HOLDER_DUMMY: no accrual

        // Compute LED partition & render (highlight only if A/B/C holds)
        apportionCounts3(hold_us, counts);
        renderRing3(leds, counts, holder);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


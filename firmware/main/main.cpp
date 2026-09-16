// main.cpp — ESP32 sensor module entry point.
//
// Wiring only: check the bay against the baked catalogue, bring the link up,
// then let the publisher decide what is due. The seeker and bay logic lives
// in publisher.cpp and the wire format in include/link/SensorLink.hpp.

#include "board_config.hpp"
#include "mission_data.generated.hpp"
#include "publisher.hpp"
#include "sensor_uart.hpp"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
constexpr const char* kTag = "sensor";

uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "ESP32 sensor module starting");
    ESP_LOGI(kTag, "baked mission data: %d tracks x %d nodes, %d stores in catalogue",
             mission_data::kTrackCount, mission_data::kNodeCount,
             mission_data::kStoreCount);

    const mission_data::Store* loaded = mission_data::findStore(board::kLoadedStore);
    if (loaded == nullptr)
    {
        // A typo here would otherwise surface as silently wrong ballistics on
        // the flight computer, so fail loudly instead.
        ESP_LOGE(kTag, "loaded store \"%s\" is not in the catalogue, halting",
                 board::kLoadedStore);
        return;
    }

    ESP_LOGI(kTag, "bay: %s mass=%.3f drag=%.4f lift=%.4f hitR=%.1f",
             loaded->name, loaded->mass, loaded->drag, loaded->lift,
             board::kStoreHitRadiusM);

    sensor_uart::Uart uart;
    if (!uart.begin())
    {
        ESP_LOGE(kTag, "link failed to start, halting");
        return;
    }

    publisher::Publisher pub(uart, *loaded);

    ESP_LOGI(kTag, "publishing: %d tracks at %d Hz, mission time scaled x%.0f",
             mission_data::kTrackCount, board::kDetectionHz,
             static_cast<double>(board::kTimeScale));

    // The publisher is driven by wall-clock milliseconds rather than a tick
    // count, so the cadence does not drift with the scheduler.
    while (true)
    {
        pub.tick(nowMs());
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

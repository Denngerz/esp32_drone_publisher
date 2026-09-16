// main.cpp — ESP32 sensor module entry point.
//
// Skeleton: reports the mission data baked in at build time and the store the
// bay is configured with. The UART transport and the publishing loop are
// added on top of this.

#include "board_config.hpp"
#include "mission_data.generated.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
constexpr const char* kTag = "sensor";
}

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

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

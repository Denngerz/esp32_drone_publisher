// main.cpp — ESP32 sensor board entry point.
//
// The board carries two modules that are separate boxes on a real airframe:
// the seeker and the payload bay. Each gets its own UART and its own task, so
// neither can stall or corrupt the other, and the flight computer sees two
// links that fail independently. Wiring only lives here; the modules are in
// seeker.cpp and bay.cpp, and the wire format in include/link/SensorLink.hpp.

#include "bay.hpp"
#include "board_config.hpp"
#include "mission_data.generated.hpp"
#include "seeker.hpp"
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

// Both modules are driven by wall-clock milliseconds rather than a tick
// count, so their cadence does not drift with the scheduler.
void seekerTask(void* arg)
{
    auto* module = static_cast<seeker::Seeker*>(arg);
    while (true)
    {
        module->tick(nowMs());
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void bayTask(void* arg)
{
    auto* module = static_cast<bay::Bay*>(arg);
    while (true)
    {
        module->tick(nowMs());
        // The bay speaks once a second, so it can afford to look far less
        // often than the seeker and leave the CPU to it.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "ESP32 sensor board starting");
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

    // Static storage, not locals: the tasks below outlive this function, and
    // app_main parks rather than returning so that stays true either way.
    static sensor_uart::Uart seekerUart(board::kSeekerUart, board::kSeekerTxPin,
                                        board::kSeekerRxPin, "seeker");
    static sensor_uart::Uart bayUart(board::kBayUart, board::kBayTxPin,
                                     board::kBayRxPin, "bay");

    if (!seekerUart.begin())
    {
        ESP_LOGE(kTag, "seeker link failed to start, halting");
        return;
    }
    if (!bayUart.begin())
    {
        ESP_LOGE(kTag, "bay link failed to start, halting");
        return;
    }

    static seeker::Seeker seekerModule(seekerUart);
    static bay::Bay       bayModule(bayUart, *loaded);

    ESP_LOGI(kTag, "seeker: %d tracks at %d Hz, mission time scaled x%.0f",
             mission_data::kTrackCount, board::kDetectionHz,
             static_cast<double>(board::kTimeScale));
    ESP_LOGI(kTag, "bay: %s repeated every %d ms",
             loaded->name, board::kAmmoRepeatMs);

    xTaskCreate(seekerTask, "seeker", 4096, &seekerModule, 5, nullptr);
    xTaskCreate(bayTask,    "bay",    4096, &bayModule,    5, nullptr);

    // Nothing left to do here, but returning would end the main task while
    // the two modules keep running, which makes the log harder to read than
    // simply parking.
    while (true)
        vTaskDelay(portMAX_DELAY);
}

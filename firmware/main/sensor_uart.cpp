#include "sensor_uart.hpp"
#include "board_config.hpp"

#include "link/SensorLink.hpp"

#include <esp_log.h>

namespace sensor_uart
{

namespace
{
constexpr const char* kTag = "uart";
}

bool Uart::begin()
{
    uart_config_t cfg = {};
    cfg.baud_rate  = board::kLinkBaud;
    cfg.data_bits  = UART_DATA_8_BITS;      // 8N1, matching the Pi's cfmakeraw
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(port_,
                                        board::kLinkBufSz,
                                        board::kLinkBufSz,
                                        0, nullptr, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(kTag, "%s: uart_driver_install failed: %s",
                 label_, esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(port_, &cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(kTag, "%s: uart_param_config failed: %s",
                 label_, esp_err_to_name(err));
        return false;
    }

    // Routing through the GPIO matrix is what makes the pin choice free. It
    // matters most for UART1, whose reset-default pins are the SPI flash.
    err = uart_set_pin(port_, txPin_, rxPin_,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(kTag, "%s: uart_set_pin failed: %s",
                 label_, esp_err_to_name(err));
        return false;
    }

    ready_ = true;
    ESP_LOGI(kTag, "%s link up: UART%d tx=%d rx=%d @ %d baud",
             label_, port_, txPin_, rxPin_, board::kLinkBaud);
    return true;
}

void Uart::send(uint8_t type, const void* payload, uint8_t payloadLen)
{
    if (!ready_) return;

    uint8_t frame[280];
    size_t n = sensor_link::encode(type, payload, payloadLen, frame);

    // Blocking write with no timeout would stall the publish cadence if the
    // peer stopped draining; the driver's buffer absorbs normal bursts and a
    // full buffer means the link is gone, which dropping a frame reports
    // honestly enough.
    uart_write_bytes(port_, reinterpret_cast<const char*>(frame), n);
}

} // namespace sensor_uart

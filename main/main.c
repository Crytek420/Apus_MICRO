#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "crsf_protocol.h"
#include "servo_control.h"

static const char *TAG = "FlightController";

/**
 * @brief Map CRSF channel value (172-1811) to control input (-1000 to +1000)
 */
static int16_t map_channel_to_control(uint16_t channel_value)
{
    // CRSF: 172 = min, 992 = center, 1811 = max
    // Map to: -1000 = min, 0 = center, +1000 = max
    int16_t centered = channel_value - 992;
    int16_t mapped = (centered * 1000) / 819; // 819 = (1811-992)

    // Clamp to range
    if (mapped < -1000)
        mapped = -1000;
    if (mapped > 1000)
        mapped = 1000;

    return mapped;
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 Flying Wing Flight Controller Starting...");

    // Initialize servos
    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();

    // Start CRSF communication task (100 Hz)
    ESP_LOGI(TAG, "Starting CRSF communication task...");
    crsf_start_task();

    // Wait for first RC data
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint32_t last_log_time = 0;

    while (1)
    {
        // Get RC channel data from CRSF
        crsf_channels_t channels;
        if (crsf_get_channels(&channels))
        {
            // Map RC channels to control inputs
            // Channel 0 = Aileron (Roll)
            // Channel 1 = Elevator (Pitch)
            // Channel 2 = Throttle (not used for elevons)
            // Channel 3 = Rudder (Yaw)

            control_input_t control = {
                .roll = map_channel_to_control(channels.channels[0]),
                .pitch = map_channel_to_control(channels.channels[1]),
                .yaw = map_channel_to_control(channels.channels[3])};

            // Apply elevon mixer
            servo_elevon_mixer(&control);

            // Log RC values at 1 Hz
            uint32_t now = xTaskGetTickCount();
            if ((now - last_log_time) >= pdMS_TO_TICKS(1000))
            {
                ESP_LOGI(TAG, "RC Sticks: Roll=%d Pitch=%d Throttle=%d Yaw=%d",
                         channels.channels[0], channels.channels[1],
                         channels.channels[2], channels.channels[3]);
                ESP_LOGI(TAG, "Switches: SA=%d SB=%d SC=%d SD=%d SE=%d",
                         channels.channels[4], channels.channels[5],
                         channels.channels[6], channels.channels[7],
                         channels.channels[8]);
                ESP_LOGI(TAG, "Aux: CH10=%d CH11=%d CH12=%d",
                         channels.channels[9], channels.channels[10],
                         channels.channels[11]);
                ESP_LOGI(TAG, "Controls: Roll=%d Pitch=%d Yaw=%d",
                         control.roll, control.pitch, control.yaw);
                last_log_time = now;
            }
        }
        else
        {
            // No RC signal - set servos to neutral for safety
            servo_set_neutral();
            ESP_LOGW(TAG, "No RC signal - servos in neutral");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz control loop
    }
}

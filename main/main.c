#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "crsf_protocol.h"
#include "servo_control.h"
#include "pi_mavlink.h"
#include "wt901b_imu.h"
#include "esc_control.h"

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
    ESP_LOGI(TAG, "ESP32-S3 Flying Wing Flight Controller Starting...");

    // Initialize servos
    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();

    // Initialize ESC
    ESP_LOGI(TAG, "Initializing ESC...");
    esc_init();

    // Initialize MAVLink for Raspberry Pi Zero 2W communication
    ESP_LOGI(TAG, "Initializing Pi MAVLink interface (UART2)...");
    pi_mavlink_init();

    // Start MAVLink task
    ESP_LOGI(TAG, "Starting MAVLink communication task...");
    pi_mavlink_start_task();

    // Initialize WT901B IMU
    ESP_LOGI(TAG, "Initializing WT901B IMU...");
    esp_err_t imu_ret = wt901b_init();
    if (imu_ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Starting WT901B IMU task...");
        wt901b_start_task();
    }
    else
    {
        ESP_LOGW(TAG, "WT901B IMU not found - continuing without IMU");
    }

    // Initialize internal temperature sensor
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&temp_config, &temp_sensor) == ESP_OK) {
        temperature_sensor_enable(temp_sensor);
        ESP_LOGI(TAG, "Temperature sensor initialized");
    } else {
        ESP_LOGW(TAG, "Temperature sensor init failed - continuing without it");
        temp_sensor = NULL;
    }

    // Start CRSF communication task (100 Hz)
    ESP_LOGI(TAG, "Starting CRSF communication task...");
    crsf_start_task();

    // Wait for first RC data
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Arm ESC (safe to do after RC is initialized)
    ESP_LOGI(TAG, "Arming ESC...");
    esc_arm();
    esc_set_filter_time(100); // 100ms filter time for smooth throttle response

    ESP_LOGI(TAG, "Initialization complete - starting main control loop");

    uint32_t last_log_time = 0;
    uint32_t last_telemetry_time = 0;
    uint32_t last_imu_time = 0;
    uint32_t last_stats_time = 0;
    float last_cycle_us = 0.0f;

    while (1)
    {
        int64_t loop_start_us = esp_timer_get_time();
        uint32_t now = xTaskGetTickCount();

        // Get RC channel data from CRSF
        crsf_channels_t channels;
        bool rc_valid = crsf_get_channels(&channels);

        if (rc_valid)
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

            // Control ESC from throttle channel (channel 2)
            // Use a switch channel to control direction (e.g., channel 4 - SA switch)
            // If channel 4 > mid point (992), use reverse, otherwise forward
            if (channels.channels[4] > CRSF_CHANNEL_VALUE_MID)
            {
                esc_set_direction(ESC_DIRECTION_REVERSE);
            }
            else
            {
                esc_set_direction(ESC_DIRECTION_FORWARD);
            }

            uint16_t throttle = esc_map_crsf_to_throttle(channels.channels[2]);
            esc_set_throttle(throttle);

            // Send RC channels to Raspberry Pi at 20 Hz via MAVLink
            if ((now - last_telemetry_time) >= pdMS_TO_TICKS(50))
            {
                // Send RC channels
                pi_mavlink_send_rc_channels(&channels, false);

                last_telemetry_time = now;
            }

            // Log RC values at 1 Hz
            if ((now - last_log_time) >= pdMS_TO_TICKS(1000))
            {
                uint16_t esc_throttle = esc_get_throttle();
                ESP_LOGI(TAG, "RC Sticks: Roll=%d Pitch=%d Throttle=%d Yaw=%d (ESC: %d%%)",
                         channels.channels[0], channels.channels[1],
                         channels.channels[2], channels.channels[3],
                         esc_throttle / 10);
                ESP_LOGI(TAG, "Switches: SA=%d SB=%d SC=%d SD=%d SE=%d",
                         channels.channels[4], channels.channels[5],
                         channels.channels[6], channels.channels[7],
                         channels.channels[8]);
                ESP_LOGI(TAG, "Aux: CH10=%d CH11=%d CH12=%d",
                         channels.channels[9], channels.channels[10],
                         channels.channels[11]);
                ESP_LOGI(TAG, "Controls: Roll=%d Pitch=%d Yaw=%d",
                         control.roll, control.pitch, control.yaw);

                // Log IMU data
                wt901b_data_t imu_data;
                if (wt901b_get_data(&imu_data))
                {
                    ESP_LOGI(TAG, "IMU: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                             imu_data.roll, imu_data.pitch, imu_data.yaw);
                }

                last_log_time = now;
            }
        }
        else
        {
            // No RC signal - cut servos/ESC to a safe state immediately every
            // loop iteration. This must not be gated behind the 1 Hz log timer,
            // otherwise the ESC keeps driving the last commanded throttle for
            // up to ~1s after the CRSF staleness timeout already fired.
            servo_set_neutral();
            esc_emergency_stop();

            if ((now - last_log_time) >= pdMS_TO_TICKS(1000))
            {
                ESP_LOGW(TAG, "No RC signal - servos in neutral, ESC stopped");

                // Send failsafe status to Pi
                crsf_channels_t dummy_channels = {0};
                pi_mavlink_send_rc_channels(&dummy_channels, true);

                last_log_time = now;
            }
        }

        // Send IMU data to Raspberry Pi at 20 Hz via MAVLink (always, regardless of RC)
        if ((now - last_imu_time) >= pdMS_TO_TICKS(50))
        {
            wt901b_data_t imu_data;
            if (wt901b_get_data(&imu_data))
            {
                pi_mavlink_send_imu(&imu_data);
            }
            last_imu_time = now;
        }

        // Measure loop body duration before the sleep
        last_cycle_us = (float)(esp_timer_get_time() - loop_start_us);

        // Send ESP32 hardware stats to Pi at 1 Hz
        if ((now - last_stats_time) >= pdMS_TO_TICKS(1000))
        {
            float temp_c = 0.0f;
            if (temp_sensor != NULL) {
                temperature_sensor_get_celsius(temp_sensor, &temp_c);
            }
            float free_heap_kb = (float)esp_get_free_heap_size() / 1024.0f;
            pi_mavlink_send_esp_stats(temp_c, free_heap_kb, last_cycle_us);
            last_stats_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz control loop
    }
}
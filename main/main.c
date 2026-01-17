#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "crsf_protocol.h"
#include "servo_control.h"
#include "pi_uart.h"
#include "wt901b_imu.h"
#include "esc_control.h"

static const char *TAG = "FlightController";

// Temporarily disable verbose logging for IMU debugging
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR

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

    // Initialize UART for Raspberry Pi Zero 2W communication
    ESP_LOGI(TAG, "Initializing Pi UART interface (UART2)...");
    pi_uart_init();

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

    // Start CRSF communication task (100 Hz)
    ESP_LOGI(TAG, "Starting CRSF communication task...");
    crsf_start_task();

    // Wait for first RC data
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Arm ESC (safe to do after RC is initialized)
    ESP_LOGI(TAG, "Arming ESC...");
    esc_arm();
    esc_set_filter_time(100); // 100ms filter time for smooth throttle response

    // Send test message to Pi via UART2
    pi_uart_send_string("Hello from ESP32-S3!\n");
    ESP_LOGI(TAG, "Test message sent to Pi via UART2");

    uint32_t last_log_time = 0;
    uint32_t last_pi_send_time = 0;
    uint32_t last_imu_log_time = 0;
    uint32_t test_counter = 0;

    while (1)
    {
        // Check for incoming data from Pi (only log if meaningful data received)
        uint8_t rx_buffer[64];
        int bytes_received = pi_uart_receive(rx_buffer, sizeof(rx_buffer) - 1);
        if (bytes_received > 0)
        {
            rx_buffer[bytes_received] = '\0';
            // Only log if we received printable characters (filter noise from open RX pin)
            bool has_printable = false;
            for (int i = 0; i < bytes_received; i++)
            {
                if (rx_buffer[i] >= 0x20 && rx_buffer[i] <= 0x7E)
                {
                    has_printable = true;
                    break;
                }
            }
            if (has_printable && bytes_received > 1)
            {
                ESP_LOGI(TAG, "Received from Pi: %s", (char *)rx_buffer);
            }
        }

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

            // Control ESC from throttle channel (channel 2)
            uint16_t throttle = esc_map_crsf_to_throttle(channels.channels[2]);
            esc_set_throttle(throttle);

            // Send channel data to Raspberry Pi Zero 2W at 50 Hz
            uint32_t now = xTaskGetTickCount();
            if ((now - last_pi_send_time) >= pdMS_TO_TICKS(20))
            {
                // Send test message every second
                if (test_counter % 50 == 0)
                {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "Test #%lu: CH0=%d CH1=%d\n",
                             test_counter / 50, channels.channels[0], channels.channels[1]);
                    pi_uart_send_string(buffer);
                }
                test_counter++;

                pi_uart_send_channels(&channels);
                last_pi_send_time = now;
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
            // No RC signal - set servos to neutral for safety
            uint32_t now = xTaskGetTickCount();
            if ((now - last_log_time) >= pdMS_TO_TICKS(1000))
            {
                // Send test message even without RC signal
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "No RC signal - Test #%lu\n", test_counter++);
                pi_uart_send_string(buffer);

                servo_set_neutral();
                esc_emergency_stop();
                ESP_LOGW(TAG, "No RC signal - servos in neutral, ESC stopped");
                last_log_time = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz control loop
    }
}

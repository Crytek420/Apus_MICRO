#include "pi_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PI_UART";

/**
 * @brief Calculate simple XOR checksum
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Initialize UART interface for Raspberry Pi Zero 2W communication
 */
esp_err_t pi_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = PI_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Configuring UART%d for Raspberry Pi Zero 2W", PI_UART_NUM);
    ESP_LOGI(TAG, "PIN Configuration: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             PI_UART_TX_PIN, PI_UART_RX_PIN, PI_UART_BAUDRATE);

    // Configure UART parameters
    esp_err_t ret = uart_param_config(PI_UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(PI_UART_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install UART driver
    ret = uart_driver_install(PI_UART_NUM, PI_UART_BUF_SIZE,
                              PI_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Pi UART initialized successfully");
    ESP_LOGI(TAG, "ESP32-S3 ready to communicate with Raspberry Pi Zero 2W");
    ESP_LOGI(TAG, "Connection: ESP32 TX(GPIO%d) -> Pi RX(GPIO15), ESP32 RX(GPIO%d) -> Pi TX(GPIO14)",
             PI_UART_TX_PIN, PI_UART_RX_PIN);

    return ESP_OK;
}

/**
 * @brief Send CRSF channel data to Raspberry Pi Zero 2W
 */
esp_err_t pi_uart_send_channels(const crsf_channels_t *channels)
{
    if (channels == NULL)
    {
        ESP_LOGE(TAG, "Invalid channel data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare data packet
    pi_data_packet_t packet;
    packet.header[0] = 0xAA;
    packet.header[1] = 0x55;

    // Copy channel data
    memcpy(packet.channels, channels->channels, sizeof(packet.channels));

    // Calculate checksum over channels only
    packet.checksum = calculate_checksum((uint8_t *)packet.channels, sizeof(packet.channels));

    // Send packet
    int bytes_written = uart_write_bytes(PI_UART_NUM, (const char *)&packet, sizeof(packet));

    if (bytes_written != sizeof(packet))
    {
        ESP_LOGW(TAG, "Failed to send complete packet: %d/%d bytes sent",
                 bytes_written, sizeof(packet));
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent %d bytes to Pi: CH0=%d CH1=%d CH2=%d CH3=%d",
             bytes_written,
             channels->channels[0],
             channels->channels[1],
             channels->channels[2],
             channels->channels[3]);

    return ESP_OK;
}

/**
 * @brief Check if valid data has been received from Pi
 */
int pi_uart_receive(uint8_t *buffer, size_t max_len)
{
    if (buffer == NULL || max_len == 0)
    {
        return 0;
    }

    // Check available data
    size_t available = 0;
    esp_err_t ret = uart_get_buffered_data_len(PI_UART_NUM, &available);

    if (ret != ESP_OK || available == 0)
    {
        return 0;
    }

    // Read available data (up to max_len)
    size_t read_len = (available < max_len) ? available : max_len;
    int bytes_read = uart_read_bytes(PI_UART_NUM, buffer, read_len, 0);

    if (bytes_read > 0)
    {
        ESP_LOGD(TAG, "Received %d bytes from Pi", bytes_read);
    }

    return bytes_read;
}

/**
 * @brief Send a string to Raspberry Pi Zero 2W
 */
esp_err_t pi_uart_send_string(const char *str)
{
    if (str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(str);
    int bytes_written = uart_write_bytes(PI_UART_NUM, str, len);

    if (bytes_written != (int)len)
    {
        ESP_LOGW(TAG, "Failed to send complete string: %d/%d bytes sent",
                 bytes_written, (int)len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent string to Pi: %s", str);
    return ESP_OK;
}

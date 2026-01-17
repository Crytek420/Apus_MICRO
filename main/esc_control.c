#include "esc_control.h"
#include "crsf_protocol.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

static const char *TAG = "ESC";

/* LEDC Configuration for ESC */
#define ESC_LEDC_TIMER LEDC_TIMER_1
#define ESC_LEDC_MODE LEDC_LOW_SPEED_MODE
#define ESC_LEDC_CHANNEL LEDC_CHANNEL_4
#define ESC_LEDC_DUTY_RES LEDC_TIMER_14_BIT // 14-bit resolution

/* State Variables */
static float g_filtered_throttle = 0.0f; // Current filtered throttle (0-1000)
static float g_target_throttle = 0.0f;   // Target throttle from input (0-1000)
static uint16_t g_filter_time_ms = ESC_DEFAULT_FILTER_TIME_MS;
static bool g_initialized = false;

/**
 * @brief Convert microseconds to LEDC duty cycle
 */
static uint32_t esc_us_to_duty(uint16_t pulse_width_us)
{
    // Period = 20ms = 20000us at 50Hz
    // Duty cycle = (pulse_width / period) * (2^14 - 1)
    uint32_t duty = (pulse_width_us * ((1 << ESC_LEDC_DUTY_RES) - 1)) / 20000;
    return duty;
}

/**
 * @brief Set ESC PWM pulse width directly
 */
static void esc_set_pulse_us(uint16_t pulse_width_us)
{
    // Clamp to safe range for Hobbywing 40A V2
    if (pulse_width_us < ESC_MIN_PULSEWIDTH_US)
        pulse_width_us = ESC_MIN_PULSEWIDTH_US;
    if (pulse_width_us > ESC_MAX_PULSEWIDTH_US)
        pulse_width_us = ESC_MAX_PULSEWIDTH_US;

    uint32_t duty = esc_us_to_duty(pulse_width_us);
    ESP_ERROR_CHECK(ledc_set_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL));
}

/**
 * @brief Update low-pass filter (called periodically)
 * Implements first-order exponential low-pass filter
 */
static void esc_update_filter(void)
{
    if (!g_initialized)
        return;

    // Calculate filter coefficient (alpha)
    // alpha = dt / (tau + dt), where tau = filter_time_ms
    // Assuming update rate of ~50Hz (20ms)
    const float dt_ms = 20.0f; // Update period in ms
    float tau = (float)g_filter_time_ms;
    float alpha = dt_ms / (tau + dt_ms);

    // Apply exponential filter
    // filtered = filtered + alpha * (target - filtered)
    g_filtered_throttle += alpha * (g_target_throttle - g_filtered_throttle);

    // Convert filtered throttle (0-1000) to pulse width (1000-2000us)
    uint16_t pulse_us = ESC_MIN_PULSEWIDTH_US +
                        (uint16_t)((g_filtered_throttle * (ESC_MAX_PULSEWIDTH_US - ESC_MIN_PULSEWIDTH_US)) / 1000.0f);

    esc_set_pulse_us(pulse_us);
}

/**
 * @brief ESC control task - runs filter update at 50Hz
 */
static void esc_control_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz update rate
    uint32_t iteration_count = 0;

    while (1)
    {
        esc_update_filter();

        // Log PWM values every second (50 iterations at 50Hz)
        if (iteration_count % 50 == 0)
        {
            uint16_t pulse_us = ESC_MIN_PULSEWIDTH_US +
                                (uint16_t)((g_filtered_throttle * (ESC_MAX_PULSEWIDTH_US - ESC_MIN_PULSEWIDTH_US)) / 1000.0f);
            ESP_LOGI(TAG, "Throttle: %d%% | PWM: %dus | Target: %.0f | Filtered: %.0f | Filter time: %dms",
                     (uint16_t)(g_filtered_throttle / 10), pulse_us, g_target_throttle, g_filtered_throttle, g_filter_time_ms);
        }

        iteration_count++;
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/**
 * @brief Initialize ESC PWM output
 */
esp_err_t esc_init(void)
{
    ESP_LOGI(TAG, "Initializing ESC on GPIO %d", ESC_OUTPUT_PIN);

    // Configure LEDC timer for ESC
    ledc_timer_config_t timer_conf = {
        .speed_mode = ESC_LEDC_MODE,
        .duty_resolution = ESC_LEDC_DUTY_RES,
        .timer_num = ESC_LEDC_TIMER,
        .freq_hz = ESC_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure LEDC channel for ESC
    ledc_channel_config_t channel_conf = {
        .gpio_num = ESC_OUTPUT_PIN,
        .speed_mode = ESC_LEDC_MODE,
        .channel = ESC_LEDC_CHANNEL,
        .timer_sel = ESC_LEDC_TIMER,
        .duty = esc_us_to_duty(ESC_MIN_PULSEWIDTH_US), // Start at minimum (safe)
        .hpoint = 0};
    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize state variables
    g_filtered_throttle = 0.0f;
    g_target_throttle = 0.0f;
    g_initialized = true;

    ESP_LOGI(TAG, "ESC initialized successfully");
    ESP_LOGI(TAG, "PWM: %dHz, Pulse range: %d-%dus, Filter time: %dms",
             ESC_PWM_FREQUENCY, ESC_MIN_PULSEWIDTH_US, ESC_MAX_PULSEWIDTH_US, g_filter_time_ms);

    // Create ESC control task for filter updates
    BaseType_t task_created = xTaskCreate(
        esc_control_task,
        "esc_task",
        2048,
        NULL,
        5, // Priority
        NULL);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ESC control task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ESC control task started at 50Hz");

    return ESP_OK;
}

/**
 * @brief Set ESC throttle value with low-pass filtering
 */
void esc_set_throttle(uint16_t throttle)
{
    if (!g_initialized)
    {
        ESP_LOGW(TAG, "ESC not initialized");
        return;
    }

    // Clamp throttle to valid range
    if (throttle > ESC_THROTTLE_MAX)
        throttle = ESC_THROTTLE_MAX;

    g_target_throttle = (float)throttle;
}

/**
 * @brief Set ESC throttle directly without filtering
 */
void esc_set_throttle_direct(uint16_t throttle)
{
    if (!g_initialized)
    {
        ESP_LOGW(TAG, "ESC not initialized");
        return;
    }

    // Clamp throttle to valid range
    if (throttle > ESC_THROTTLE_MAX)
        throttle = ESC_THROTTLE_MAX;

    // Set both target and filtered to same value (bypass filter)
    g_target_throttle = (float)throttle;
    g_filtered_throttle = (float)throttle;

    // Immediate update
    uint16_t pulse_us = ESC_MIN_PULSEWIDTH_US +
                        (uint16_t)((throttle * (ESC_MAX_PULSEWIDTH_US - ESC_MIN_PULSEWIDTH_US)) / 1000);
    esc_set_pulse_us(pulse_us);
}

/**
 * @brief Configure the low-pass filter time constant
 */
void esc_set_filter_time(uint16_t time_ms)
{
    if (time_ms < 10)
    {
        ESP_LOGW(TAG, "Filter time too small, using minimum 10ms");
        time_ms = 10;
    }
    if (time_ms > 1000)
    {
        ESP_LOGW(TAG, "Filter time too large, using maximum 1000ms");
        time_ms = 1000;
    }

    g_filter_time_ms = time_ms;
    ESP_LOGI(TAG, "Filter time set to %dms", time_ms);
}

/**
 * @brief Get current filtered throttle value
 */
uint16_t esc_get_throttle(void)
{
    return (uint16_t)g_filtered_throttle;
}

/**
 * @brief Arm the ESC
 */
void esc_arm(void)
{
    if (!g_initialized)
    {
        ESP_LOGW(TAG, "ESC not initialized");
        return;
    }

    ESP_LOGI(TAG, "Arming ESC...");

    // Send arming pulse (minimum throttle) for 2 seconds
    esc_set_throttle_direct(0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "ESC armed and ready");
}

/**
 * @brief Emergency stop
 */
void esc_emergency_stop(void)
{
    ESP_LOGW(TAG, "EMERGENCY STOP!");

    if (g_initialized)
    {
        g_target_throttle = 0.0f;
        g_filtered_throttle = 0.0f;
        esc_set_pulse_us(ESC_MIN_PULSEWIDTH_US);
    }
}

/**
 * @brief Map CRSF channel value to ESC throttle
 */
uint16_t esc_map_crsf_to_throttle(uint16_t channel_value)
{
    // CRSF: 172 = min, 992 = center, 1811 = max
    // For throttle, we want: 172 = 0%, 1811 = 100%

    // Clamp input to valid CRSF range
    if (channel_value < CRSF_CHANNEL_VALUE_MIN)
        channel_value = CRSF_CHANNEL_VALUE_MIN;
    if (channel_value > CRSF_CHANNEL_VALUE_MAX)
        channel_value = CRSF_CHANNEL_VALUE_MAX;

    // Map to throttle range (0-1000)
    // throttle = (channel - min) * 1000 / (max - min)
    int32_t throttle = ((int32_t)channel_value - CRSF_CHANNEL_VALUE_MIN) * 1000;
    throttle = throttle / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN);

    // Clamp to output range
    if (throttle < 0)
        throttle = 0;
    if (throttle > ESC_THROTTLE_MAX)
        throttle = ESC_THROTTLE_MAX;

    return (uint16_t)throttle;
}

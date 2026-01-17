#include "servo_control.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "SERVO";

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR

/* LEDC Timer and Channel Configuration */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT // 14-bit resolution
#define LEDC_FREQUENCY 50               // 50 Hz

/* Channel assignments */
static const ledc_channel_t servo_channels[SERVO_COUNT] = {
    LEDC_CHANNEL_0, // Left outer
    LEDC_CHANNEL_1, // Left inner
    LEDC_CHANNEL_2, // Right inner
    LEDC_CHANNEL_3  // Right outer
};

static const int servo_pins[SERVO_COUNT] = {
    SERVO_LEFT_OUTER_PIN,
    SERVO_LEFT_INNER_PIN,
    SERVO_RIGHT_INNER_PIN,
    SERVO_RIGHT_OUTER_PIN};

/**
 * @brief Convert microseconds to LEDC duty cycle
 */
static uint32_t servo_us_to_duty(uint16_t pulse_width_us)
{
    // Period = 20ms = 20000us at 50Hz
    // Duty cycle = (pulse_width / period) * (2^14 - 1)
    uint32_t duty = (pulse_width_us * ((1 << LEDC_DUTY_RES) - 1)) / 20000;
    return duty;
}

/**
 * @brief Initialize servo PWM outputs
 */
esp_err_t servo_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Configure LEDC channels for each servo
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ledc_channel_config_t channel_conf = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_MODE,
            .channel = servo_channels[i],
            .timer_sel = LEDC_TIMER,
            .duty = servo_us_to_duty(SERVO_MID_PULSEWIDTH_US),
            .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
    }

    ESP_LOGI(TAG, "Servos initialized on pins: %d, %d, %d, %d",
             SERVO_LEFT_OUTER_PIN, SERVO_LEFT_INNER_PIN,
             SERVO_RIGHT_INNER_PIN, SERVO_RIGHT_OUTER_PIN);

    // Set all servos to neutral
    servo_set_neutral();

    return ESP_OK;
}

/**
 * @brief Set servo position in microseconds
 */
esp_err_t servo_set_pulse_us(servo_channel_t channel, uint16_t pulse_width_us)
{
    if (channel >= SERVO_COUNT)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp to safe range
    if (pulse_width_us < SERVO_MIN_PULSEWIDTH_US)
        pulse_width_us = SERVO_MIN_PULSEWIDTH_US;
    if (pulse_width_us > SERVO_MAX_PULSEWIDTH_US)
        pulse_width_us = SERVO_MAX_PULSEWIDTH_US;

    uint32_t duty = servo_us_to_duty(pulse_width_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, servo_channels[channel], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, servo_channels[channel]));

    return ESP_OK;
}

/**
 * @brief Set servo position in degrees
 */
esp_err_t servo_set_angle(servo_channel_t channel, int8_t angle)
{
    // Clamp angle to -90 to +90
    if (angle < -90)
        angle = -90;
    if (angle > 90)
        angle = 90;

    // Map angle to pulse width
    // -90° = 500us, 0° = 1500us, +90° = 2500us
    uint16_t pulse_width_us = SERVO_MID_PULSEWIDTH_US + (angle * 1000 / 90);

    return servo_set_pulse_us(channel, pulse_width_us);
}

/**
 * @brief Elevon mixer for flying wing with differential thrust for yaw
 *
 * Flying Wing Mixer Logik:
 * - Roll: Verschränkung der Klappen (Elevons)
 * - Pitch: Beide Klappen gleichzeitig
 * - Yaw: Differenzielle Verschränkung (asymmetrisches Roll-Moment)
 *
 * Mixing Formula:
 * Left Elevons  = +Pitch - Roll + Yaw
 * Right Elevons = +Pitch + Roll + Yaw
 */
void servo_elevon_mixer(const control_input_t *input)
{
    // Scale inputs from -1000/+1000 to servo angles
    // Aggressiver Mixer: höhere Ausschläge für schnellere Reaktion
    float pitch_angle = (input->pitch / 1000.0f) * 50.0f; // Pitch: ±50° (war 30°)
    float roll_angle = (input->roll / 1000.0f) * 70.0f;   // Roll: ±70° (war 45°)
    float yaw_angle = (input->yaw / 1000.0f) * 25.0f;     // Yaw: ±25° (war 15°)

    // Calculate elevon deflections
    float left_outer_angle = pitch_angle - roll_angle + yaw_angle;
    float left_inner_angle = pitch_angle - roll_angle + yaw_angle;
    float right_inner_angle = pitch_angle + roll_angle + yaw_angle;
    float right_outer_angle = pitch_angle + roll_angle + yaw_angle;

    // Clamp to servo limits (-90 to +90) - Servos erreichen oft Limit
    left_outer_angle = fmaxf(-90.0f, fminf(90.0f, left_outer_angle));
    left_inner_angle = fmaxf(-90.0f, fminf(90.0f, left_inner_angle));
    right_inner_angle = fmaxf(-90.0f, fminf(90.0f, right_inner_angle));
    right_outer_angle = fmaxf(-90.0f, fminf(90.0f, right_outer_angle));

    // Set servo positions
    servo_set_angle(SERVO_LEFT_OUTER, (int8_t)left_outer_angle);
    servo_set_angle(SERVO_LEFT_INNER, (int8_t)left_inner_angle);
    servo_set_angle(SERVO_RIGHT_INNER, (int8_t)right_inner_angle);
    servo_set_angle(SERVO_RIGHT_OUTER, (int8_t)right_outer_angle);
}

/**
 * @brief Set all servos to neutral position
 */
void servo_set_neutral(void)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servo_set_pulse_us((servo_channel_t)i, SERVO_MID_PULSEWIDTH_US);
    }
    ESP_LOGI(TAG, "All servos set to neutral");
}

/**
 * @brief Emergency: Disable all servos
 */
void servo_disable_all(void)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ledc_stop(LEDC_MODE, servo_channels[i], 0);
    }
    ESP_LOGW(TAG, "All servos disabled!");
}

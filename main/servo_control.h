#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include "esp_err.h"

/* Servo Configuration */
#define SERVO_COUNT 4

/* Servo Pin Definitions (PWM capable GPIOs on ESP32-S3)
 * Using GPIO 9, 10, 11, 12 - these don't have ADC1 channels
 * ADC1 channels (GPIO 1-8) kept free for voltage/current sensing */
#define SERVO_LEFT_OUTER_PIN    9    // Linke Tragfläche, äußere Klappe
#define SERVO_LEFT_INNER_PIN    10   // Linke Tragfläche, innere Klappe
#define SERVO_RIGHT_INNER_PIN   11   // Rechte Tragfläche, innere Klappe
#define SERVO_RIGHT_OUTER_PIN   12   // Rechte Tragfläche, äußere Klappe

/* PWM Configuration for SG90 */
#define SERVO_PWM_FREQUENCY     50      // 50Hz for SG90
#define SERVO_MIN_PULSEWIDTH_US 500     // 0.5ms = -90°
#define SERVO_MAX_PULSEWIDTH_US 2500    // 2.5ms = +90°
#define SERVO_MID_PULSEWIDTH_US 1500    // 1.5ms = 0°

/* Servo Channel Mapping */
typedef enum {
    SERVO_LEFT_OUTER = 0,
    SERVO_LEFT_INNER = 1,
    SERVO_RIGHT_INNER = 2,
    SERVO_RIGHT_OUTER = 3
} servo_channel_t;

/* Control Surface Inputs (-1000 to +1000) */
typedef struct {
    int16_t roll;       // Roll: positive = right wing down
    int16_t pitch;      // Pitch: positive = nose up
    int16_t yaw;        // Yaw: positive = nose right
} control_input_t;

/* Function Prototypes */

/**
 * @brief Initialize servo PWM outputs
 */
esp_err_t servo_init(void);

/**
 * @brief Set servo position in microseconds
 * @param channel Servo channel (0-3)
 * @param pulse_width_us Pulse width in microseconds (500-2500)
 */
esp_err_t servo_set_pulse_us(servo_channel_t channel, uint16_t pulse_width_us);

/**
 * @brief Set servo position in degrees
 * @param channel Servo channel (0-3)
 * @param angle Angle in degrees (-90 to +90)
 */
esp_err_t servo_set_angle(servo_channel_t channel, int8_t angle);

/**
 * @brief Elevon mixer for flying wing
 * Mixes roll, pitch, and yaw inputs to control surfaces
 * @param input Control inputs (-1000 to +1000)
 */
void servo_elevon_mixer(const control_input_t *input);

/**
 * @brief Set all servos to neutral position
 */
void servo_set_neutral(void);

/**
 * @brief Emergency: Disable all servos
 */
void servo_disable_all(void);

#endif // SERVO_CONTROL_H

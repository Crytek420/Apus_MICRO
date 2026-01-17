#ifndef ESC_CONTROL_H
#define ESC_CONTROL_H

#include <stdint.h>
#include "esp_err.h"

/* ESC Configuration for Hobbywing 40A V2 */
#define ESC_PWM_FREQUENCY     50        // 50Hz standard for hobby ESCs
#define ESC_MIN_PULSEWIDTH_US 1000      // 1.0ms = 0% throttle (motor off)
#define ESC_MAX_PULSEWIDTH_US 2000      // 2.0ms = 100% throttle (full power)
#define ESC_ARM_PULSEWIDTH_US 1000      // Arming pulse width

/* ESC Pin Definition */
#define ESC_OUTPUT_PIN        13        // GPIO 13 (free pin, PWM capable)

/* ESC Throttle Range (input from RC) */
#define ESC_THROTTLE_MIN      0         // 0% throttle
#define ESC_THROTTLE_MAX      1000      // 100% throttle (scaled from CRSF input)

/* Low-Pass Filter Configuration */
#define ESC_DEFAULT_FILTER_TIME_MS  100  // Default filter time constant in milliseconds

/* Function Prototypes */

/**
 * @brief Initialize ESC PWM output
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t esc_init(void);

/**
 * @brief Set ESC throttle value with low-pass filtering
 * @param throttle Throttle value (0-1000, where 0=off, 1000=full power)
 * @note The output will be filtered based on the configured filter time
 */
void esc_set_throttle(uint16_t throttle);

/**
 * @brief Set ESC throttle directly without filtering (for arming/calibration)
 * @param throttle Throttle value (0-1000)
 */
void esc_set_throttle_direct(uint16_t throttle);

/**
 * @brief Configure the low-pass filter time constant
 * @param time_ms Filter time constant in milliseconds (higher = smoother but slower response)
 * @note Typical values: 50-200ms. Default is 100ms.
 */
void esc_set_filter_time(uint16_t time_ms);

/**
 * @brief Get current filtered throttle value
 * @return Current throttle value (0-1000)
 */
uint16_t esc_get_throttle(void);

/**
 * @brief Arm the ESC (send minimum throttle for arming sequence)
 * @note Call this after initialization and before applying throttle
 */
void esc_arm(void);

/**
 * @brief Emergency stop - immediately set throttle to zero
 */
void esc_emergency_stop(void);

/**
 * @brief Map CRSF channel value to ESC throttle (172-1811 → 0-1000)
 * @param channel_value CRSF channel value (172-1811)
 * @return Throttle value (0-1000)
 */
uint16_t esc_map_crsf_to_throttle(uint16_t channel_value);

#endif // ESC_CONTROL_H

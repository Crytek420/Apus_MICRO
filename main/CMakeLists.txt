idf_component_register(SRCS "pi_mavlink.c" "main.c" "crsf_protocol.c" "servo_control.c" "wt901b_imu.c" "esc_control.c"
                    PRIV_REQUIRES spi_flash driver esp_timer
                    INCLUDE_DIRS "")

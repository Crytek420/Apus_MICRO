idf_component_register(SRCS "main.c" "crsf_protocol.c" "servo_control.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES driver)

idf_component_register(SRCS "x_drive_esp.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos esp_common driver esp_timer lib_ps5_esp32 mpu6050 stepper)

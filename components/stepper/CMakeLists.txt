idf_component_register(SRCS "stepper.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos driver esp_timer)

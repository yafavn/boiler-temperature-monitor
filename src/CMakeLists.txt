idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES 
                        onewire_bus 
                        ds18b20 
                        esp_adc 
                        esp_driver_gpio
                        esp_driver_rmt
)

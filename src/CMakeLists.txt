idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." "../include"
                    PRIV_REQUIRES 
                        onewire_bus 
                        ds18b20 
                        esp_adc 
                        esp_driver_gpio
                        esp_driver_rmt
                        esp_wifi        
                        esp_event       
                        nvs_flash       
                        esp_netif       
                        lwip
                        mqtt
                        esp_http_client
                        esp_https_ota
                        app_update      # מכיל את esp_ota_ops.h
)
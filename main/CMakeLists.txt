idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "freertos" "log" "driver" "spi_flash" "bt" "arduino-audio-tools" "ESP32-A2DP" "newlib" "esp_adc" "hal" "nvs_flash" "esp_common")
                    set(COMPONENT_REQUIRES "newlib")
                    


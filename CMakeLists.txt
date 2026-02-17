# lib/nrf24l01/CMakeLists.txt
idf_component_register(SRCS "./src/nrf24l01.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver freertos esp_rom log)
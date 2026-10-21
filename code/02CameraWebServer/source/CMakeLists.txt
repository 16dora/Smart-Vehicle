idf_component_register(SRCS "main.cpp" "camera_web.cpp" "motor_control.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES esp_http_server esp_wifi esp_event esp_netif nvs_flash
             esp_timer esp_psram esp32-camera esp_driver_ledc esp_driver_gpio esp_driver_i2c
    EMBED_TXTFILES "../web/index.html")
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES)

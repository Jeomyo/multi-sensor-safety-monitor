idf_component_register(SRCS "sensor_average.c" "buzzer_control.c" "dht11_sensor.c" "rgb_led_control.c" "app_main.c"
                             "led_control.c"
                             "wifi_manager.c"
                             "mqtt_manager.c"
                             "web_server.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp_wifi esp_http_server nvs_flash mqtt esp_driver_gpio json)

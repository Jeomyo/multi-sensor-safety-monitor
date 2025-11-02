#include "rgb_led_control.h"
#include "config.h"
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rgb_led";

esp_err_t rgb_led_control_init(void)
{
    ESP_LOGI(TAG, "RGB LED 초기화: R=GPIO%d, G=GPIO%d, B=GPIO%d", 
             RGB_LED_R_PIN, RGB_LED_G_PIN, RGB_LED_B_PIN);
    
    // R 핀 초기화 (공통 음극: 1=ON, 0=OFF)
    gpio_reset_pin(RGB_LED_R_PIN);
    gpio_set_direction(RGB_LED_R_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RGB_LED_R_PIN, 0); // OFF (LOW)
    
    // G 핀 초기화
    gpio_reset_pin(RGB_LED_G_PIN);
    gpio_set_direction(RGB_LED_G_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RGB_LED_G_PIN, 0); // OFF (LOW)
    
    // B 핀 초기화
    gpio_reset_pin(RGB_LED_B_PIN);
    gpio_set_direction(RGB_LED_B_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RGB_LED_B_PIN, 0); // OFF (LOW)
    
    ESP_LOGI(TAG, "RGB LED 초기화 완료");
    return ESP_OK;
}

esp_err_t rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    // 공통 음극 RGB LED: 1(HIGH) = ON, 0(LOW) = OFF
    // 값이 127보다 크면 1(HIGH)로 설정하여 ON
    gpio_set_level(RGB_LED_R_PIN, (r > 127) ? 1 : 0);  // 값이 크면 1(HIGH) = ON
    gpio_set_level(RGB_LED_G_PIN, (g > 127) ? 1 : 0);
    gpio_set_level(RGB_LED_B_PIN, (b > 127) ? 1 : 0);
    
    ESP_LOGI(TAG, "RGB LED 색상 설정: R=%d, G=%d, B=%d", r, g, b);
    return ESP_OK;
}

esp_err_t rgb_led_off(void)
{
    // 공통 음극: 0(LOW) = OFF (모든 핀을 LOW로 설정)
    gpio_set_level(RGB_LED_R_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_G_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_B_PIN, 0); // OFF (LOW)
    return ESP_OK;
}

esp_err_t rgb_led_set_red(void)
{
    // 공통 음극: 1(HIGH) = ON (해당 색상 핀만 HIGH)
    gpio_set_level(RGB_LED_R_PIN, 1); // ON (HIGH)
    gpio_set_level(RGB_LED_G_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_B_PIN, 0); // OFF (LOW)
    ESP_LOGI(TAG, "RGB LED: 빨강색");
    return ESP_OK;
}

esp_err_t rgb_led_set_green(void)
{
    // 공통 음극: 1(HIGH) = ON
    gpio_set_level(RGB_LED_R_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_G_PIN, 1); // ON (HIGH)
    gpio_set_level(RGB_LED_B_PIN, 0); // OFF (LOW)
    ESP_LOGI(TAG, "RGB LED: 초록색");
    return ESP_OK;
}

esp_err_t rgb_led_set_blue(void)
{
    // 공통 음극: 1(HIGH) = ON
    gpio_set_level(RGB_LED_R_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_G_PIN, 0); // OFF (LOW)
    gpio_set_level(RGB_LED_B_PIN, 1); // ON (HIGH)
    ESP_LOGI(TAG, "RGB LED: 파랑색");
    return ESP_OK;
}


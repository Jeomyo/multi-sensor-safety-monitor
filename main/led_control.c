#include "led_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "led_control";
static bool led_state = false;

esp_err_t led_control_init(void)
{
    ESP_LOGI(TAG, "LED 초기화: GPIO %d", LED_GPIO_PIN);
    
    gpio_reset_pin(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO_PIN, 0); // 초기 상태: OFF
    led_state = false;
    
    ESP_LOGI(TAG, "LED 초기화 완료");
    return ESP_OK;
}

esp_err_t led_control_on(void)
{
    gpio_set_level(LED_GPIO_PIN, 1);
    led_state = true;
    ESP_LOGI(TAG, "LED 켜짐");
    return ESP_OK;
}

esp_err_t led_control_off(void)
{
    gpio_set_level(LED_GPIO_PIN, 0);
    led_state = false;
    ESP_LOGI(TAG, "LED 꺼짐");
    return ESP_OK;
}

esp_err_t led_control_toggle(void)
{
    led_state = !led_state;
    gpio_set_level(LED_GPIO_PIN, led_state ? 1 : 0);
    ESP_LOGI(TAG, "LED 토글: %s", led_state ? "ON" : "OFF");
    return ESP_OK;
}

bool led_control_get_state(void)
{
    return led_state;
}


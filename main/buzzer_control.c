#include "buzzer_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "buzzer";
static bool buzzer_state = false;

esp_err_t buzzer_control_init(void)
{
    ESP_LOGI(TAG, "부저 초기화: GPIO %d", BUZZER_GPIO_PIN);
    
    gpio_reset_pin(BUZZER_GPIO_PIN);
    gpio_set_direction(BUZZER_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO_PIN, 0); // 초기 상태: OFF
    buzzer_state = false;
    
    ESP_LOGI(TAG, "부저 초기화 완료");
    return ESP_OK;
}

esp_err_t buzzer_on(void)
{
    gpio_set_level(BUZZER_GPIO_PIN, 1);
    buzzer_state = true;
    ESP_LOGI(TAG, "부저 켜짐");
    return ESP_OK;
}

esp_err_t buzzer_off(void)
{
    gpio_set_level(BUZZER_GPIO_PIN, 0);
    buzzer_state = false;
    ESP_LOGI(TAG, "부저 꺼짐");
    return ESP_OK;
}

esp_err_t buzzer_toggle(void)
{
    buzzer_state = !buzzer_state;
    gpio_set_level(BUZZER_GPIO_PIN, buzzer_state ? 1 : 0);
    ESP_LOGI(TAG, "부저 토글: %s", buzzer_state ? "ON" : "OFF");
    return ESP_OK;
}

bool buzzer_get_state(void)
{
    return buzzer_state;
}


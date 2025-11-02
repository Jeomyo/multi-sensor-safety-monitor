#ifndef LED_CONTROL_INTERNAL_H
#define LED_CONTROL_INTERNAL_H

// 테스트를 위한 내부 인터페이스
// GPIO 호출을 함수 포인터로 추상화하여 모의 가능하게 함

#include <stdbool.h>
#include "esp_err.h"

// GPIO 함수 타입 정의
typedef esp_err_t (*gpio_reset_pin_func_t)(int pin);
typedef esp_err_t (*gpio_set_direction_func_t)(int pin, int mode);
typedef esp_err_t (*gpio_set_level_func_t)(int pin, int level);

// GPIO 함수 포인터 (테스트 시 교체 가능)
extern gpio_reset_pin_func_t led_gpio_reset_pin;
extern gpio_set_direction_func_t led_gpio_set_direction;
extern gpio_set_level_func_t led_gpio_set_level;

#endif // LED_CONTROL_INTERNAL_H



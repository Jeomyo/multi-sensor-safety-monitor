#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief LED 제어 모듈 초기화
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t led_control_init(void);

/**
 * @brief LED 켜기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t led_control_on(void);

/**
 * @brief LED 끄기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t led_control_off(void);

/**
 * @brief LED 토글
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t led_control_toggle(void);

/**
 * @brief 현재 LED 상태 조회
 * @return true: 켜짐, false: 꺼짐
 */
bool led_control_get_state(void);

#endif // LED_CONTROL_H


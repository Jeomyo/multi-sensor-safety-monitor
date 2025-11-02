#ifndef RGB_LED_CONTROL_H
#define RGB_LED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief RGB LED 제어 모듈 초기화
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_control_init(void);

/**
 * @brief RGB LED 색상 설정
 * @param r 빨강 값 (0-255)
 * @param g 초록 값 (0-255)
 * @param b 파랑 값 (0-255)
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief RGB LED 끄기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_off(void);

/**
 * @brief RGB LED 빨강색 켜기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_set_red(void);

/**
 * @brief RGB LED 초록색 켜기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_set_green(void);

/**
 * @brief RGB LED 파랑색 켜기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t rgb_led_set_blue(void);

#endif // RGB_LED_CONTROL_H


#ifndef BUZZER_CONTROL_H
#define BUZZER_CONTROL_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 부저 제어 모듈 초기화
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t buzzer_control_init(void);

/**
 * @brief 부저 켜기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t buzzer_on(void);

/**
 * @brief 부저 끄기
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t buzzer_off(void);

/**
 * @brief 부저 토글
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t buzzer_toggle(void);

/**
 * @brief 현재 부저 상태 조회
 * @return true: 켜짐, false: 꺼짐
 */
bool buzzer_get_state(void);

#endif // BUZZER_CONTROL_H


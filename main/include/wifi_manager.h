#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief WiFi STA 모드 초기화 및 연결
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief WiFi 연결 상태 확인
 * @return true: 연결됨, false: 연결 안됨
 */
bool wifi_manager_is_connected(void);

/**
 * @brief 현재 할당된 IP 주소 가져오기
 * @param ip_str IP 주소를 저장할 문자열 버퍼 (최소 16바이트)
 * @param len 버퍼 길이
 * @return true: IP 주소 있음, false: IP 주소 없음
 */
bool wifi_manager_get_ip(char *ip_str, size_t len);

#endif // WIFI_MANAGER_H


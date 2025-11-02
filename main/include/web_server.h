#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

/**
 * @brief HTTP 웹서버 초기화 및 시작
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t web_server_init(void);

/**
 * @brief HTTP 웹서버 정리
 */
void web_server_deinit(void);

#endif // WEB_SERVER_H


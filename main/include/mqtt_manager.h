#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief MQTT 클라이언트 초기화 및 시작
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t mqtt_manager_init(void);

/**
 * @brief MQTT 연결 상태 확인
 * @return true: 연결됨, false: 연결 안됨
 */
bool mqtt_manager_is_connected(void);

/**
 * @brief MQTT 메시지 발행
 * @param topic 토픽 이름
 * @param data 발행할 데이터
 * @param len 데이터 길이
 * @param qos QoS 레벨 (0, 1, 2)
 * @param retain Retain 플래그
 * @return 메시지 ID (성공), -1 (실패)
 */
int mqtt_manager_publish(const char* topic, const char* data, int len, int qos, int retain);

/**
 * @brief MQTT 토픽 구독
 * @param topic 토픽 이름
 * @param qos QoS 레벨
 * @return 메시지 ID (성공), -1 (실패)
 */
int mqtt_manager_subscribe(const char* topic, int qos);

/**
 * @brief LED 상태 변경 콜백 함수 타입
 */
typedef void (*mqtt_led_callback_t)(bool led_on);

/**
 * @brief LED 제어 메시지 콜백 등록
 * @param callback 콜백 함수
 */
void mqtt_manager_register_led_callback(mqtt_led_callback_t callback);

#endif // MQTT_MANAGER_H


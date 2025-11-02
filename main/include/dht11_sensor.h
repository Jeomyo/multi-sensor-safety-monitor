#ifndef DHT11_SENSOR_H
#define DHT11_SENSOR_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float temperature;  // 온도 (섭씨)
    float humidity;     // 습도 (%)
    bool valid;         // 데이터 유효성
} dht11_data_t;

/**
 * @brief DHT11 센서 초기화
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t dht11_init(void);

/**
 * @brief DHT11 센서에서 온도/습도 읽기
 * @param data 읽은 데이터를 저장할 구조체 포인터
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t dht11_read(dht11_data_t *data);

#endif // DHT11_SENSOR_H


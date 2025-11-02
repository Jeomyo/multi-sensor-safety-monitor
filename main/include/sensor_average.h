#ifndef SENSOR_AVERAGE_H
#define SENSOR_AVERAGE_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float temperature;
    float humidity;
} sensor_reading_t;

/**
 * @brief 센서 평균 계산 모듈 초기화
 * @return ESP_OK 성공, 그 외 오류 코드
 */
esp_err_t sensor_average_init(void);

/**
 * @brief 새로운 센서 데이터 추가
 * @param temperature 온도 값
 * @param humidity 습도 값
 */
void sensor_average_add(float temperature, float humidity);

/**
 * @brief 현재 센서 데이터 가져오기
 * @param temperature 현재 온도 출력 포인터
 * @param humidity 현재 습도 출력 포인터
 * @return true: 유효한 데이터, false: 데이터 없음
 */
bool sensor_average_get_current(float *temperature, float *humidity);

/**
 * @brief 5분 평균 센서 데이터 가져오기
 * @param temperature 평균 온도 출력 포인터
 * @param humidity 평균 습도 출력 포인터
 * @return true: 유효한 평균 데이터, false: 데이터 부족
 */
bool sensor_average_get_5min(float *temperature, float *humidity);

#endif // SENSOR_AVERAGE_H


#include "sensor_average.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "sensor_avg";

// 5분간 데이터 저장: 2초마다 읽으면 5분 = 150개
#define MAX_READINGS 150
#define AVERAGE_PERIOD_SECONDS 300  // 5분

static sensor_reading_t readings[MAX_READINGS];
static int current_index = 0;
static int reading_count = 0;
static bool initialized = false;
static float current_temperature = 0.0f;
static float current_humidity = 0.0f;

esp_err_t sensor_average_init(void)
{
    memset(readings, 0, sizeof(readings));
    current_index = 0;
    reading_count = 0;
    initialized = true;
    ESP_LOGI(TAG, "센서 평균 계산 모듈 초기화 완료");
    return ESP_OK;
}

void sensor_average_add(float temperature, float humidity)
{
    if (!initialized) {
        return;
    }
    
    // 현재 값 저장
    current_temperature = temperature;
    current_humidity = humidity;
    
    // 원형 버퍼에 저장
    readings[current_index].temperature = temperature;
    readings[current_index].humidity = humidity;
    
    current_index = (current_index + 1) % MAX_READINGS;
    
    if (reading_count < MAX_READINGS) {
        reading_count++;
    }
}

bool sensor_average_get_current(float *temperature, float *humidity)
{
    if (!initialized || temperature == NULL || humidity == NULL) {
        return false;
    }
    
    if (reading_count == 0) {
        return false;
    }
    
    *temperature = current_temperature;
    *humidity = current_humidity;
    return true;
}

bool sensor_average_get_5min(float *temperature, float *humidity)
{
    if (!initialized || temperature == NULL || humidity == NULL) {
        return false;
    }
    
    // 최소 10개 이상의 데이터가 있어야 평균 계산
    if (reading_count < 10) {
        return false;
    }
    
    float sum_temp = 0.0f;
    float sum_hum = 0.0f;
    int count = (reading_count < MAX_READINGS) ? reading_count : MAX_READINGS;
    
    for (int i = 0; i < count; i++) {
        sum_temp += readings[i].temperature;
        sum_hum += readings[i].humidity;
    }
    
    *temperature = sum_temp / count;
    *humidity = sum_hum / count;
    
    return true;
}


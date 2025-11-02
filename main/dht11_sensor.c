#include "dht11_sensor.h"
#include "config.h"
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "dht11";

// DHT11 타이밍 상수 (마이크로초)
#define DHT11_START_SIGNAL_LOW_TIME    20000   // 20ms LOW
#define DHT11_START_SIGNAL_HIGH_TIME   30      // 30us HIGH
#define DHT11_RESPONSE_LOW_TIME        80      // 80us LOW
#define DHT11_RESPONSE_HIGH_TIME_MIN   50      // 최소 50us HIGH
#define DHT11_RESPONSE_HIGH_TIME_MAX   80      // 최대 80us HIGH
#define DHT11_DATA_LOW_TIME            50      // 50us LOW
#define DHT11_DATA_HIGH_TIME_0_MIN     26      // 0 비트: 26-28us HIGH
#define DHT11_DATA_HIGH_TIME_0_MAX     28
#define DHT11_DATA_HIGH_TIME_1_MIN     70      // 1 비트: 70us HIGH
#define DHT11_DATA_HIGH_TIME_1_MAX     75

// 마이크로초 단위 지연 (CPU 주파수에 따라 조정 필요할 수 있음)
static void dht11_delay_us(uint32_t us)
{
    ets_delay_us(us);
}

// GPIO 레벨 읽기 (마이크로초 단위 타임아웃)
static int dht11_read_bit(void)
{
    int count = 0;
    
    // LOW 레벨 대기 (약 50us, 최대 80us)
    while (gpio_get_level(DHT_GPIO_PIN) == 0 && count < 80) {
        dht11_delay_us(1);
        count++;
    }
    
    if (count >= 80) {
        return -1; // 타임아웃
    }
    
    // HIGH 레벨 유지 시간 측정 (최대 100us)
    count = 0;
    while (gpio_get_level(DHT_GPIO_PIN) == 1 && count < 100) {
        dht11_delay_us(1);
        count++;
    }
    
    // HIGH 시간에 따라 0 또는 1 판단
    // 0 비트: 26-28us (약 30us 미만), 1 비트: 70us (약 50us 이상)
    // 중간값인 40us를 기준으로 판단
    if (count >= 40) {
        return 1; // 1 비트 (70us)
    }
    return 0; // 0 비트 (26-28us)
}

esp_err_t dht11_init(void)
{
    ESP_LOGI(TAG, "DHT11 초기화: GPIO %d", DHT_GPIO_PIN);
    
    gpio_reset_pin(DHT_GPIO_PIN);
    gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO_PIN, 1); // 초기 상태 HIGH
    
    ESP_LOGI(TAG, "DHT11 초기화 완료");
    return ESP_OK;
}

esp_err_t dht11_read(dht11_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data_bytes[5] = {0};
    
    // 최대 3번 재시도
    for (int retry = 0; retry < 3; retry++) {
        // 재시도 전 대기 (첫 시도 제외)
        if (retry > 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
        }
        
        // 핀을 출력 모드로 설정
        gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DHT_GPIO_PIN, 1);
        dht11_delay_us(100); // 안정화 대기
        
        // 시작 신호: 18-20ms LOW (약간 짧게)
        gpio_set_level(DHT_GPIO_PIN, 0);
        dht11_delay_us(18000); // 18ms
        
        // 입력 모드로 전환하고 풀업 활성화
        gpio_set_direction(DHT_GPIO_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode(DHT_GPIO_PIN, GPIO_PULLUP_ONLY);
        
        // HIGH로 변할 때까지 대기 (최대 40us)
        uint32_t timeout = 0;
        while (gpio_get_level(DHT_GPIO_PIN) == 0 && timeout < 50) {
            dht11_delay_us(1);
            timeout++;
        }
        
        if (timeout >= 50) {
            if (retry < 2) {
                continue; // 재시도
            }
            ESP_LOGE(TAG, "DHT11 응답 신호 타임아웃 (LOW->HIGH 전환 실패)");
            data->valid = false;
            return ESP_ERR_TIMEOUT;
        }
        
        // HIGH 유지 대기 (최대 80us)
        timeout = 0;
        while (gpio_get_level(DHT_GPIO_PIN) == 1 && timeout < 90) {
            dht11_delay_us(1);
            timeout++;
        }
        
        if (timeout >= 90) {
            if (retry < 2) {
                continue; // 재시도
            }
            ESP_LOGE(TAG, "DHT11 응답 신호 타임아웃 (HIGH 유지)");
            data->valid = false;
            return ESP_ERR_TIMEOUT;
        }
        
        // LOW로 변할 때까지 대기 (데이터 시작 신호, 최대 80us)
        timeout = 0;
        while (gpio_get_level(DHT_GPIO_PIN) == 0 && timeout < 90) {
            dht11_delay_us(1);
            timeout++;
        }
        
        if (timeout >= 90) {
            if (retry < 2) {
                continue; // 재시도
            }
            ESP_LOGE(TAG, "DHT11 응답 신호 타임아웃 (LOW 유지)");
            data->valid = false;
            return ESP_ERR_TIMEOUT;
        }
        
        // 데이터 바이트 초기화
        for (int i = 0; i < 5; i++) {
            data_bytes[i] = 0;
        }
        
        // 데이터 40비트 읽기
        bool read_success = true;
        for (int i = 0; i < 5; i++) {
            for (int j = 7; j >= 0; j--) {
                int bit = dht11_read_bit();
                if (bit < 0) {
                    read_success = false;
                    if (retry < 2) {
                        break; // 재시도를 위해 루프 종료
                    }
                    ESP_LOGE(TAG, "DHT11 데이터 읽기 실패 (바이트 %d, 비트 %d)", i, j);
                    data->valid = false;
                    return ESP_ERR_TIMEOUT;
                }
                data_bytes[i] |= (bit << j);
            }
            if (!read_success && retry < 2) {
                break; // 바깥 루프도 종료
            }
        }
        
        // 비트 읽기 실패 시 재시도
        if (!read_success && retry < 2) {
            continue;
        }
        
        uint8_t checksum = data_bytes[0] + data_bytes[1] + data_bytes[2] + data_bytes[3];
        if (checksum == data_bytes[4]) {
            // 데이터 파싱
            data->humidity = (float)data_bytes[0] + (float)data_bytes[1] / 10.0;
            data->temperature = (float)data_bytes[2] + (float)data_bytes[3] / 10.0;
            data->valid = true;
            
            // 성공 메시지 출력 (재시도 여부와 관계없이)
            ESP_LOGI(TAG, "DHT11 읽기 성공: 온도=%.1f°C, 습도=%.1f%%", 
                     data->temperature, data->humidity);
            
            return ESP_OK;
        }
        
        // 체크섬 오류 - 재시도
        if (retry < 2) {
            // 재시도는 루프 시작 부분에서 처리됨
        }
    }
    
    // 모든 재시도 실패
    ESP_LOGE(TAG, "DHT11 읽기 실패: 체크섬 오류 (3회 시도 모두 실패)");
    data->valid = false;
    return ESP_ERR_INVALID_CRC;
}


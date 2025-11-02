/* ESP32 WiFi, WebServer & MQTT Example - 모듈화 버전
 *
 * 이 예제는 WiFi 연결, 웹서버 기능, MQTT 통신을 모듈화하여 구현했습니다.
 * 각 모듈은 독립적으로 관리되며 재사용이 가능합니다.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include <time.h>

#include "led_control.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "web_server.h"
#include "rgb_led_control.h"
#include "dht11_sensor.h"
#include "buzzer_control.h"
#include "sensor_average.h"
#include "config.h"

static const char *TAG = "main";

// MQTT LED 제어 콜백 함수
static void mqtt_led_callback(bool led_on)
{
    if (led_on) {
        led_control_on();
        ESP_LOGI(TAG, "MQTT로 LED 켜기");
        
        // 상태 토픽에 발행
        if (mqtt_manager_is_connected()) {
            char status_msg[50];
            snprintf(status_msg, sizeof(status_msg), "{\"led\":true}");
            mqtt_manager_publish(MQTT_TOPIC_STATUS, status_msg, strlen(status_msg), 0, 1);
        }
    } else {
        led_control_off();
        ESP_LOGI(TAG, "MQTT로 LED 끄기");
        
        // 상태 토픽에 발행
        if (mqtt_manager_is_connected()) {
            char status_msg[50];
            snprintf(status_msg, sizeof(status_msg), "{\"led\":false}");
            mqtt_manager_publish(MQTT_TOPIC_STATUS, status_msg, strlen(status_msg), 0, 1);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 WiFi, WebServer & MQTT 시작 (모듈화 버전)");
    
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS 초기화 완료");

    // 센서 초기화
    ESP_LOGI(TAG, "RGB LED 모듈 초기화 중...");
    ESP_ERROR_CHECK(rgb_led_control_init());
    
    ESP_LOGI(TAG, "DHT11 센서 초기화 중...");
    ESP_ERROR_CHECK(dht11_init());
    
    ESP_LOGI(TAG, "센서 평균 계산 모듈 초기화 중...");
    ESP_ERROR_CHECK(sensor_average_init());
    
    ESP_LOGI(TAG, "부저 모듈 초기화 중...");
    ESP_ERROR_CHECK(buzzer_control_init());

    // WiFi 연결
    ESP_LOGI(TAG, "WiFi 모듈 초기화 중... SSID: %s", WIFI_SSID);
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_LOGI(TAG, "WiFi 연결 완료");

    // MQTT 클라이언트 시작
    ESP_LOGI(TAG, "MQTT 모듈 초기화 중... 브로커: %s", MQTT_BROKER_URL);
    mqtt_manager_register_led_callback(mqtt_led_callback);
    ESP_ERROR_CHECK(mqtt_manager_init());
    ESP_LOGI(TAG, "MQTT 클라이언트 시작 완료");

    // 웹서버 시작
    ESP_LOGI(TAG, "HTTP 웹서버 모듈 초기화 중... 포트: %d", HTTP_SERVER_PORT);
    ESP_ERROR_CHECK(web_server_init());
    ESP_LOGI(TAG, "웹서버 시작 완료");

    // IP 주소 가져오기
    char ip_address[16] = "0.0.0.0";
    if (wifi_manager_get_ip(ip_address, sizeof(ip_address))) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✅ 모든 모듈 초기화 완료!");
        ESP_LOGI(TAG, "🌐 브라우저에서 접속: http://%s", ip_address);
        ESP_LOGI(TAG, "📡 MQTT 브로커: %s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
        ESP_LOGI(TAG, "📨 MQTT 토픽:");
        ESP_LOGI(TAG, "   - %s (LED 제어)", MQTT_TOPIC_LED);
        ESP_LOGI(TAG, "   - %s (상태)", MQTT_TOPIC_STATUS);
        ESP_LOGI(TAG, "   - %s (센서 데이터 발행)", MQTT_TOPIC_SENSOR);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✅ 모든 모듈 초기화 완료!");
        ESP_LOGI(TAG, "🌐 IP 주소 할당 대기 중... (시리얼 모니터에서 확인)");
        ESP_LOGI(TAG, "📡 MQTT 브로커: %s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
        ESP_LOGI(TAG, "========================================");
    }
    ESP_LOGI(TAG, "🔧 센서 테스트 시작!");
    ESP_LOGI(TAG, "========================================");

    // RGB LED 초기화 (끄기)
    rgb_led_off();
    
    static int dht11_counter = 0; // DHT11 읽기 카운터
    
    // 메인 루프 - 센서 테스트
    while (1) {
        
        // DHT11 온습도 읽기 (2초마다)
        if (dht11_counter % 2 == 0) {
            dht11_data_t dht_data;
            esp_err_t ret = dht11_read(&dht_data);
            if (ret == ESP_OK && dht_data.valid) {
                ESP_LOGI(TAG, "🌡️  온도: %.1f°C, 💧 습도: %.1f%%", 
                         dht_data.temperature, dht_data.humidity);
                
                // 평균 계산 모듈에 데이터 추가
                sensor_average_add(dht_data.temperature, dht_data.humidity);
                
                // MQTT로 센서 데이터 발행
                if (mqtt_manager_is_connected()) {
                    int64_t timestamp_us = esp_timer_get_time();
                    int64_t timestamp_s = timestamp_us / 1000000; // 초 단위
                    
                    char sensor_json[200];
                    int len = snprintf(sensor_json, sizeof(sensor_json),
                        "{"
                        "\"deviceId\":\"esp32-001\","
                        "\"temperature\":%.1f,"
                        "\"humidity\":%.1f,"
                        "\"timestamp\":%lld,"
                        "\"valid\":true"
                        "}",
                        dht_data.temperature,
                        dht_data.humidity,
                        timestamp_s);
                    
                    if (len > 0 && len < sizeof(sensor_json)) {
                        mqtt_manager_publish(MQTT_TOPIC_SENSOR, sensor_json, len, 0, 0);
                        ESP_LOGI(TAG, "📡 MQTT 센서 데이터 발행: %s", sensor_json);
                    } else {
                        ESP_LOGE(TAG, "센서 데이터 JSON 생성 실패 (버퍼 크기 초과)");
                    }
                } else {
                    ESP_LOGW(TAG, "MQTT 연결되지 않음 - 센서 데이터 발행 건너뜀");
                }
            } else {
                ESP_LOGW(TAG, "DHT11 읽기 실패 또는 데이터 무효");
            }
        }
        dht11_counter++;
        
        // 부저 테스트: 3초마다 ON/OFF 토글
        static int buzzer_counter = 0;
        static bool buzzer_on_state = false;
        if (buzzer_counter % 3 == 0) {
            if (!buzzer_on_state) {
                buzzer_on();
                buzzer_on_state = true;
                ESP_LOGI(TAG, "🔊 부저: ON");
            } else {
                buzzer_off();
                buzzer_on_state = false;
                ESP_LOGI(TAG, "🔇 부저: OFF");
            }
        }
        buzzer_counter++;
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1초 대기
    }
}

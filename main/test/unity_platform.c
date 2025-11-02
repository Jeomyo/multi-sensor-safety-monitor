#include "unity.h"
#include "esp_log.h"

// Unity 테스트 플랫폼 설정
void setUp(void) {
    // 테스트 전 설정
}

void tearDown(void) {
    // 테스트 후 정리
}

// ESP32 로그를 Unity 출력으로 리다이렉트
void unity_print(const char* message) {
    ESP_LOGI("TEST", "%s", message);
}



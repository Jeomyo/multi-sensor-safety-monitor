/**
 * @file test_led_control_simple.c
 * @brief LED 제어 모듈 간단한 테스트
 * 
 * ESP-IDF 테스트 프레임워크를 사용한 통합 테스트
 * 실제 GPIO 없이도 동작 로직을 검증
 */

#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "esp_err.h"
#include "driver/gpio.h"

// 테스트 대상 모듈 포함
#include "led_control.h"

TEST_CASE("LED 제어 초기화 테스트", "[led_control]")
{
    esp_err_t ret = led_control_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    
    // 초기 상태는 OFF
    TEST_ASSERT_FALSE(led_control_get_state());
}

TEST_CASE("LED 켜기 테스트", "[led_control]")
{
    led_control_init();
    
    esp_err_t ret = led_control_on();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(led_control_get_state());
}

TEST_CASE("LED 끄기 테스트", "[led_control]")
{
    led_control_init();
    led_control_on(); // 먼저 켜기
    
    esp_err_t ret = led_control_off();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FALSE(led_control_get_state());
}

TEST_CASE("LED 토글 테스트", "[led_control]")
{
    led_control_init();
    
    // 초기 상태 저장
    bool initial_state = led_control_get_state();
    
    // 토글 후 상태 변경 확인
    esp_err_t ret = led_control_toggle();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_EQUAL(initial_state, led_control_get_state());
    
    // 다시 토글하면 원래 상태로
    ret = led_control_toggle();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(initial_state, led_control_get_state());
}

TEST_CASE("LED 상태 조회 테스트", "[led_control]")
{
    led_control_init();
    
    // 초기 상태는 OFF
    TEST_ASSERT_FALSE(led_control_get_state());
    
    // 켜면 true
    led_control_on();
    TEST_ASSERT_TRUE(led_control_get_state());
    
    // 끄면 false
    led_control_off();
    TEST_ASSERT_FALSE(led_control_get_state());
}

TEST_CASE("LED 상태 전환 시퀀스 테스트", "[led_control]")
{
    led_control_init();
    
    // OFF -> ON -> OFF -> ON 순서 테스트
    TEST_ASSERT_FALSE(led_control_get_state());
    
    led_control_on();
    TEST_ASSERT_TRUE(led_control_get_state());
    
    led_control_off();
    TEST_ASSERT_FALSE(led_control_get_state());
    
    led_control_on();
    TEST_ASSERT_TRUE(led_control_get_state());
}



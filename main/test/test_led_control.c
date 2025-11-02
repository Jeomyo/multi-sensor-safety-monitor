#include <unity.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// Mock GPIO 함수들
static int gpio_reset_pin_called = 0;
static int gpio_set_direction_called = 0;
static int gpio_set_level_called = 0;
static int gpio_level_value = -1;
static int gpio_pin_used = -1;

// GPIO 모의 함수
void gpio_reset_pin(int pin) {
    gpio_reset_pin_called++;
    gpio_pin_used = pin;
}

void gpio_set_direction(int pin, int mode) {
    gpio_set_direction_called++;
    gpio_pin_used = pin;
}

int gpio_set_level(int pin, int level) {
    gpio_set_level_called++;
    gpio_pin_used = pin;
    gpio_level_value = level;
    return ESP_OK;
}

// 테스트 대상 모듈 포함 (GPIO 의존성 제거를 위해 조건부 컴파일 필요)
// 실제로는 led_control.c에서 GPIO 호출을 매크로로 래핑하거나
// 함수 포인터를 사용하여 모의 가능하도록 해야 함

// 여기서는 테스트 구조만 보여줌
extern esp_err_t led_control_init(void);
extern esp_err_t led_control_on(void);
extern esp_err_t led_control_off(void);
extern esp_err_t led_control_toggle(void);
extern bool led_control_get_state(void);

void setUp(void) {
    // 각 테스트 전 초기화
    gpio_reset_pin_called = 0;
    gpio_set_direction_called = 0;
    gpio_set_level_called = 0;
    gpio_level_value = -1;
    gpio_pin_used = -1;
}

void tearDown(void) {
    // 각 테스트 후 정리
}

void test_led_control_init(void) {
    esp_err_t ret = led_control_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(gpio_reset_pin_called > 0);
    TEST_ASSERT_TRUE(gpio_set_direction_called > 0);
    TEST_ASSERT_TRUE(gpio_set_level_called > 0);
}

void test_led_control_on(void) {
    led_control_init();
    gpio_set_level_called = 0;
    
    esp_err_t ret = led_control_on();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(gpio_set_level_called > 0);
    TEST_ASSERT_EQUAL(1, gpio_level_value);
    TEST_ASSERT_TRUE(led_control_get_state());
}

void test_led_control_off(void) {
    led_control_init();
    led_control_on(); // 먼저 켜기
    gpio_set_level_called = 0;
    
    esp_err_t ret = led_control_off();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(gpio_set_level_called > 0);
    TEST_ASSERT_EQUAL(0, gpio_level_value);
    TEST_ASSERT_FALSE(led_control_get_state());
}

void test_led_control_toggle(void) {
    led_control_init();
    
    bool initial_state = led_control_get_state();
    esp_err_t ret = led_control_toggle();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_EQUAL(initial_state, led_control_get_state());
    
    // 다시 토글하면 원래 상태로
    ret = led_control_toggle();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(initial_state, led_control_get_state());
}

void test_led_control_get_state(void) {
    led_control_init();
    
    // 초기 상태는 false
    TEST_ASSERT_FALSE(led_control_get_state());
    
    // 켜면 true
    led_control_on();
    TEST_ASSERT_TRUE(led_control_get_state());
    
    // 끄면 false
    led_control_off();
    TEST_ASSERT_FALSE(led_control_get_state());
}

// 테스트 러너
void app_main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_led_control_init);
    RUN_TEST(test_led_control_on);
    RUN_TEST(test_led_control_off);
    RUN_TEST(test_led_control_toggle);
    RUN_TEST(test_led_control_get_state);
    
    UNITY_END();
}



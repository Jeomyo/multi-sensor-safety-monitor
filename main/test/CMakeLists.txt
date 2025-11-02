# 테스트 컴포넌트 설정
idf_component_register(
    SRCS "unity_platform.c" "test_led_control.c" "test_led_control_simple.c"
    INCLUDE_DIRS ".." "../include"
    PRIV_REQUIRES unity
)

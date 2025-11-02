#ifndef CONFIG_H
#define CONFIG_H

// WiFi 설정
#define WIFI_SSID              "KT_GiGA_2371"
#define WIFI_PASS              "6kg52ke294"
#define WIFI_MAXIMUM_RETRY     5

// MQTT 설정
#define MQTT_BROKER_URL        "mqtt://broker.hivemq.com"
#define MQTT_BROKER_PORT       1883
#define MQTT_CLIENT_ID         "esp32_client"
#define MQTT_TOPIC_LED         "esp32/led"       // 기존 LED 제어 (호환성)
#define MQTT_TOPIC_RGB_LED     "esp32/rgb_led"   // RGB LED 제어
#define MQTT_TOPIC_BUZZER      "esp32/buzzer"    // 부저 제어
#define MQTT_TOPIC_SENSOR      "esp32/sensor"    // 온습도 센서 데이터 발행
#define MQTT_TOPIC_STATUS      "esp32/status"    // 상태 발행

// 센서 핀 설정
// 기존 LED 핀 (호환성을 위해 유지, 현재 사용 안 함)
#define LED_GPIO_PIN           2       // 기존 LED 출력 핀 (호환성)

// RGB LED 핀 설정 (3개 핀 + GND)
#define RGB_LED_R_PIN          4       // RGB LED 빨강(R) 핀
#define RGB_LED_G_PIN          5       // RGB LED 초록(G) 핀
#define RGB_LED_B_PIN          6       // RGB LED 파랑(B) 핀
// RGB LED GND는 공통 GND에 연결

// DHT11 온습도 센서 (10K 풀업 저항 내장)
// 왼쪽부터: S(Signal), V(VCC), G(GND)
#define DHT_GPIO_PIN           7       // DHT11 Signal 핀 (S)

// 부저 센서 (3핀: VCC, GND, Signal)
#define BUZZER_GPIO_PIN        8       // 부저 Signal 핀

// HTTP 서버 설정
#define HTTP_SERVER_PORT       80

#endif // CONFIG_H


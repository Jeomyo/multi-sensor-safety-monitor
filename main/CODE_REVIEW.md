# blink_example_main.c 상세 코드 리뷰

ESP32에서 WiFi, HTTP 웹서버, MQTT를 통합하여 LED를 제어하는 펌웨어의 전체 코드 분석 문서입니다.

---

## 📋 목차
1. [헤더 파일 및 전역 변수](#1-헤더-파일-및-전역-변수)
2. [WiFi 연결 기능](#2-wifi-연결-기능)
3. [MQTT 통신 기능](#3-mqtt-통신-기능)
4. [HTTP 웹서버 기능](#4-http-웹서버-기능)
5. [LED 제어 기능](#5-led-제어-기능)
6. [메인 함수 및 실행 흐름](#6-메인-함수-및-실행-흐름)
7. [전체 동작 흐름도](#7-전체-동작-흐름도)

---

## 1. 헤더 파일 및 전역 변수

### 1.1 헤더 파일 포함 (6-24번 라인)

```6:24:main/blink_example_main.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "sdkconfig.h"
```

**설명:**
- **FreeRTOS 관련**: `FreeRTOS.h`, `task.h`, `event_groups.h` - RTOS 태스크와 이벤트 그룹(비동기 이벤트 동기화)
- **ESP-IDF 코어**: `esp_system.h`, `esp_log.h`, `nvs_flash.h` - 시스템 기능, 로깅, 비휘발성 저장소
- **WiFi**: `esp_wifi.h`, `esp_event.h` - WiFi 드라이버와 이벤트 루프
- **네트워킹**: `lwip/*` - TCP/IP 스택(LwIP)
- **HTTP 서버**: `esp_http_server.h` - 내장 HTTP 서버
- **GPIO**: `driver/gpio.h` - GPIO 제어
- **MQTT**: `mqtt_client.h` - MQTT 클라이언트 라이브러리
- **JSON**: `cJSON.h` - JSON 파싱(현재 코드에서는 미사용)

### 1.2 상수 정의 (28-41번 라인)

```28:41:main/blink_example_main.c
// WiFi 설정
#define WIFI_SSID      "KT_GiGA_2371"
#define WIFI_PASS      "6kg52ke294"
#define WIFI_MAXIMUM_RETRY  5

// MQTT 설정
#define MQTT_BROKER_URL    "mqtt://broker.hivemq.com"
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "esp32_client"
#define MQTT_TOPIC_LED     "esp32/led"
#define MQTT_TOPIC_STATUS  "esp32/status"

// LED 핀 설정
#define LED_GPIO_PIN   2
```

**설명:**
- **WiFi**: SSID와 비밀번호를 하드코딩(운영 시 menuconfig 또는 NVS 사용 권장)
- **MQTT**: 공개 브로커(`broker.hivemq.com`) 사용(테스트용)
- **토픽 구조**:
  - `esp32/led`: LED 제어 토픽(구독) - "on"/"off" 수신
  - `esp32/status`: LED 상태 발행 토픽 - `{"led":true/false}` JSON
- **LED**: GPIO 2번 핀 사용(ESP32 보드 기본 LED)

### 1.3 전역 변수 (26, 47-50번 라인)

```26:26:main/blink_example_main.c
static const char *TAG = "mqtt_webserver";
```

```47:50:main/blink_example_main.c
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool led_state = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
```

**설명:**
- **TAG**: ESP-IDF 로깅용 태그("mqtt_webserver")
- **s_wifi_event_group**: FreeRTOS 이벤트 그룹 - WiFi 연결 성공/실패 비트 플래그
- **s_retry_num**: WiFi 재연결 시도 횟수 카운터
- **led_state**: 현재 LED 상태(true=ON, false=OFF)
- **mqtt_client**: MQTT 클라이언트 핸들

---

## 2. WiFi 연결 기능

### 2.1 WiFi 이벤트 핸들러 (53-73번 라인)

```53:73:main/blink_example_main.c
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
```

**기능 설명:**
1. **WIFI_EVENT_STA_START**: WiFi STA 모드 시작 시 자동으로 `esp_wifi_connect()` 호출하여 연결 시작
2. **WIFI_EVENT_STA_DISCONNECTED**: 연결 끊김 시:
   - 재시도 횟수 미만이면 재연결 시도
   - 최대 재시도 횟수 초과 시 `WIFI_FAIL_BIT` 설정(연결 실패 표시)
3. **IP_EVENT_STA_GOT_IP**: IP 주소 할당 완료 시:
   - 할당된 IP 주소 로그 출력
   - 재시도 카운터 리셋
   - `WIFI_CONNECTED_BIT` 설정(연결 성공 표시)

**동작 흐름:**
```
시작 → WIFI_EVENT_STA_START → esp_wifi_connect() 호출
  ↓
연결 성공 → IP_EVENT_STA_GOT_IP → WIFI_CONNECTED_BIT 설정
  ↓
연결 실패 → WIFI_EVENT_STA_DISCONNECTED → 재시도 or FAIL_BIT 설정
```

### 2.2 WiFi 초기화 함수 (76-136번 라인)

```76:136:main/blink_example_main.c
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of retries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
```

**단계별 설명:**

1. **이벤트 그룹 생성 (78번 라인)**
   - WiFi 연결 성공/실패 동기화용 이벤트 그룹 생성

2. **네트워크 인터페이스 초기화 (80-82번 라인)**
   - `esp_netif_init()`: 네트워크 인터페이스 레이어 초기화
   - `esp_event_loop_create_default()`: 기본 이벤트 루프 생성
   - `esp_netif_create_default_wifi_sta()`: WiFi STA용 기본 네트워크 인터페이스 생성

3. **WiFi 드라이버 초기화 (84-85번 라인)**
   - 기본 설정으로 WiFi 드라이버 초기화

4. **이벤트 핸들러 등록 (87-98번 라인)**
   - WiFi 이벤트 전체(`ESP_EVENT_ANY_ID`)와 IP 할당 이벤트(`IP_EVENT_STA_GOT_IP`)를 `event_handler`에 연결

5. **WiFi 설정 및 시작 (100-113번 라인)**
   - SSID/비밀번호 설정
   - WPA2 PSK 인증 모드
   - PMF(Protected Management Frames) 설정(선택적)
   - STA 모드로 설정 후 WiFi 시작

6. **연결 대기 (119-123번 라인)**
   - `xEventGroupWaitBits()`로 `WIFI_CONNECTED_BIT` 또는 `WIFI_FAIL_BIT` 설정 대기
   - `portMAX_DELAY`: 무한 대기(연결 성공 또는 최대 재시도까지)

7. **결과 확인 (127-135번 라인)**
   - 연결 성공/실패 로그 출력

---

## 3. MQTT 통신 기능

### 3.1 MQTT 이벤트 핸들러 (139-205번 라인)

```139:205:main/blink_example_main.c
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_LED, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        
        // 연결 상태를 상태 토픽으로 발행
        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_STATUS, "connected", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        
        // LED 제어 메시지 처리
        if (strncmp(event->topic, MQTT_TOPIC_LED, event->topic_len) == 0) {
            if (strncmp(event->data, "on", event->data_len) == 0) {
                led_state = true;
                gpio_set_level(LED_GPIO_PIN, 1);
                ESP_LOGI(TAG, "LED turned ON via MQTT");
            } else if (strncmp(event->data, "off", event->data_len) == 0) {
                led_state = false;
                gpio_set_level(LED_GPIO_PIN, 0);
                ESP_LOGI(TAG, "LED turned OFF via MQTT");
            }
            
            // 상태 업데이트를 상태 토픽으로 발행
            char status_msg[50];
            snprintf(status_msg, sizeof(status_msg), "{\"led\":%s}", led_state ? "true" : "false");
            msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_STATUS, status_msg, 0, 1, 0);
            ESP_LOGI(TAG, "sent status update, msg_id=%d", msg_id);
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
        
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
```

**이벤트별 상세 설명:**

1. **MQTT_EVENT_CONNECTED (146-153번 라인)**
   - MQTT 브로커 연결 성공 시
   - `esp32/led` 토픽 구독(QoS 0)
   - `esp32/status` 토픽에 "connected" 발행(QoS 1, Retain 0)

2. **MQTT_EVENT_DATA (172-194번 라인)** - 핵심 기능
   - 수신한 토픽과 페이로드 로그 출력
   - 토픽이 `esp32/led`인 경우:
     - 페이로드가 "on" → LED 켜기(GPIO HIGH) + `led_state = true`
     - 페이로드가 "off" → LED 끄기(GPIO LOW) + `led_state = false`
   - 상태 변경 후 `esp32/status`에 JSON 형태로 상태 발행
     - 예: `{"led":true}` 또는 `{"led":false}`

**중요 포인트:**
- `%.*s` 포맷: `topic_len`, `data_len`만큼만 문자열 출력(널 종료 문자 불필요)
- `strncmp()`: 길이 기반 문자열 비교(안전)
- 상태 발행으로 다른 구독자들이 LED 상태 동기화 가능

### 3.2 MQTT 클라이언트 초기화 (208-220번 라인)

```208:220:main/blink_example_main.c
void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .broker.address.port = MQTT_BROKER_PORT,
        .credentials.client_id = MQTT_CLIENT_ID,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started");
}
```

**설명:**
1. **설정 구조체**: 브로커 URI, 포트, 클라이언트 ID 설정
2. **클라이언트 초기화**: `esp_mqtt_client_init()`로 핸들 생성
3. **이벤트 핸들러 등록**: 모든 MQTT 이벤트를 `mqtt_event_handler`에 연결
4. **클라이언트 시작**: 백그라운드에서 브로커 연결 시도

### 3.3 LED 상태 발행 함수 (223-231번 라인)

```223:231:main/blink_example_main.c
void publish_led_status(void)
{
    if (mqtt_client) {
        char status_msg[50];
        snprintf(status_msg, sizeof(status_msg), "{\"led\":%s}", led_state ? "true" : "false");
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_STATUS, status_msg, 0, 1, 0);
        ESP_LOGI(TAG, "Published LED status: %s, msg_id=%d", status_msg, msg_id);
    }
}
```

**기능:**
- 현재 LED 상태를 `esp32/status` 토픽에 JSON으로 발행
- **파라미터 설명**:
  - `0`: QoS 0(최대 한 번 전달)
  - `1`: Retain 1(브로커가 마지막 메시지 유지)
  - `0`: Duplicate 플래그

**참고:** 현재 코드에서는 이 함수가 정의되어 있으나 호출되지 않음. HTTP 핸들러에서 직접 발행하는 구조.

---

## 4. HTTP 웹서버 기능

### 4.1 메인 페이지 핸들러 (243-331번 라인)

```243:331:main/blink_example_main.c
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Root page requested");
    
    const char* html_page = 
        "<!DOCTYPE html>"
        ...
        "</html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}
```

**기능:**
- `GET /` 요청 시 HTML 페이지 반환
- HTML 페이지 내용:
  - LED 상태 표시 영역
  - "LED 켜기", "LED 끄기", "LED 토글" 버튼
  - 시스템 정보(WiFi SSID, 포트, MQTT 브로커/토픽)
- JavaScript:
  - `controlLED(state)`: `/led` POST 요청으로 LED 제어
  - `toggleLED()`: 현재 상태 반전
  - `updateStatus(state)`: UI 상태 업데이트
  - `window.onload`: 페이지 로드 시 `/status` GET으로 초기 상태 조회

### 4.2 LED 상태 조회 핸들러 (334-344번 라인)

```334:344:main/blink_example_main.c
static esp_err_t status_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Status requested");
    
    char response[100];
    snprintf(response, sizeof(response), "{\"state\": %s}", led_state ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}
```

**기능:**
- `GET /status` 요청 처리
- 현재 `led_state` 값을 JSON으로 반환
- 예: `{"state": true}` 또는 `{"state": false}`

### 4.3 LED 제어 핸들러 (347-394번 라인)

```347:394:main/blink_example_main.c
static esp_err_t led_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "LED control requested");
    
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    // JSON 파싱 (간단한 방식)
    char *state_str = strstr(buf, "\"state\":");
    if (state_str) {
        state_str += 8; // "state": 부분 건너뛰기
        if (strstr(state_str, "true")) {
            led_state = true;
            gpio_set_level(LED_GPIO_PIN, 1);
            ESP_LOGI(TAG, "LED turned ON via Web");
            
            // MQTT로 LED 켜기 메시지 발행
            if (mqtt_client) {
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_LED, "on", 0, 1, 0);
                ESP_LOGI(TAG, "Published LED ON to MQTT, msg_id=%d", msg_id);
            }
        } else if (strstr(state_str, "false")) {
            led_state = false;
            gpio_set_level(LED_GPIO_PIN, 0);
            ESP_LOGI(TAG, "LED turned OFF via Web");
            
            // MQTT로 LED 끄기 메시지 발행
            if (mqtt_client) {
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_LED, "off", 0, 1, 0);
                ESP_LOGI(TAG, "Published LED OFF to MQTT, msg_id=%d", msg_id);
            }
        }
    }
    
    char response[100];
    snprintf(response, sizeof(response), "{\"state\": %s, \"success\": true}", led_state ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}
```

**처리 흐름:**
1. **요청 본문 수신 (351-359번 라인)**
   - 최대 99바이트 읽기(널 종료 문자 공간 확보)
   - 타임아웃 시 408 응답

2. **JSON 파싱 (361-386번 라인)**
   - `strstr()`로 `"state"` 문자열 찾기(단순 파싱)
   - "true" 발견 → LED 켜기 + MQTT `esp32/led`에 "on" 발행
   - "false" 발견 → LED 끄기 + MQTT `esp32/led`에 "off" 발행

3. **응답 전송 (388-393번 라인)**
   - 성공 여부와 현재 상태를 JSON으로 반환

**중요 포인트:**
- HTTP 제어 시에도 MQTT에 발행하여 다른 구독자들이 상태 변화 인지 가능
- 단순 파싱 방식이므로 복잡한 JSON에는 `cJSON` 라이브러리 사용 권장

### 4.4 HTTP 서버 시작 함수 (397-436번 라인)

```397:436:main/blink_example_main.c
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);
        
        httpd_uri_t led_uri = {
            .uri       = "/led",
            .method    = HTTP_POST,
            .handler   = led_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &led_uri);
        
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
```

**설명:**
1. **서버 설정 (400-401번 라인)**
   - 기본 설정 사용(포트 80)
   - `lru_purge_enable = true`: LRU 캐시 활성화(메모리 효율)

2. **서버 시작 (404번 라인)**
   - HTTP 서버 시작

3. **URI 핸들러 등록 (407-429번 라인)**
   - `GET /` → `root_get_handler`
   - `GET /status` → `status_get_handler`
   - `POST /led` → `led_post_handler`

---

## 5. LED 제어 기능

### 5.1 LED 초기화 함수 (234-240번 라인)

```234:240:main/blink_example_main.c
void configure_led(void)
{
    ESP_LOGI(TAG, "LED configured on GPIO %d", LED_GPIO_PIN);
    gpio_reset_pin(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO_PIN, 0); // LED 끄기
    led_state = false;
}
```

**설명:**
1. **핀 리셋**: GPIO 핀을 기본 상태로 리셋
2. **출력 모드 설정**: GPIO를 출력 모드로 설정
3. **초기 상태**: LED를 끄고(0) 전역 변수 `led_state`를 `false`로 설정

**참고:** 실제 코드에서는 `led_state` 초기화가 없지만, 전역 변수는 기본적으로 `false`로 초기화됨.

---

## 6. 메인 함수 및 실행 흐름

### 6.1 app_main 함수 (438-477번 라인)

```438:477:main/blink_example_main.c
void app_main(void)
{
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // LED 초기화
    configure_led();

    // WiFi 연결
    ESP_LOGI(TAG, "ESP32 WiFi, WebServer & MQTT 시작");
    ESP_LOGI(TAG, "WiFi 연결 중... SSID: %s", WIFI_SSID);
    wifi_init_sta();

    // MQTT 클라이언트 시작
    ESP_LOGI(TAG, "MQTT 클라이언트 시작 중...");
    mqtt_app_start();

    // 웹서버 시작
    ESP_LOGI(TAG, "웹서버 시작 중...");
    httpd_handle_t server = start_webserver();
    
    if (server) {
        ESP_LOGI(TAG, "✅ 웹서버 시작 완료!");
        ESP_LOGI(TAG, "🌐 브라우저에서 접속하세요!");
        ESP_LOGI(TAG, "📡 MQTT 브로커: broker.hivemq.com");
        ESP_LOGI(TAG, "📨 토픽: esp32/led (제어), esp32/status (상태)");
    } else {
        ESP_LOGE(TAG, "❌ 웹서버 시작 실패!");
    }

    // 메인 루프
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

**실행 순서:**
1. **NVS 초기화 (441-446번 라인)**
   - 비휘발성 저장소 초기화
   - 페이지 오류 시 지우고 재초기화

2. **LED 초기화 (449번 라인)**
   - GPIO 설정 및 LED 끄기

3. **WiFi 연결 (452-454번 라인)**
   - WiFi STA 모드로 AP 연결(블로킹)

4. **MQTT 시작 (457-458번 라인)**
   - MQTT 클라이언트 초기화 및 연결 시작(비블로킹)

5. **웹서버 시작 (461-471번 라인)**
   - HTTP 서버 시작 및 URI 핸들러 등록

6. **메인 루프 (474-476번 라인)**
   - 무한 루프로 1초마다 딜레이
   - 실제 작업은 이벤트 핸들러에서 비동기 처리

---

## 7. 전체 동작 흐름도

```
[시작]
  ↓
[NVS 초기화]
  ↓
[LED 초기화] ──────────→ GPIO 2번 출력 모드, LED OFF
  ↓
[WiFi 초기화] ──────────→ AP 연결 대기 (블로킹)
  ├─→ WIFI_EVENT_STA_START → esp_wifi_connect()
  ├─→ WIFI_EVENT_STA_DISCONNECTED → 재시도 (최대 5회)
  └─→ IP_EVENT_STA_GOT_IP → WIFI_CONNECTED_BIT 설정
  ↓
[MQTT 클라이언트 시작] ──→ broker.hivemq.com 연결 (비블로킹)
  ├─→ MQTT_EVENT_CONNECTED → esp32/led 구독, esp32/status에 "connected" 발행
  └─→ MQTT_EVENT_DATA (esp32/led 수신) → LED 제어 + esp32/status에 상태 발행
  ↓
[HTTP 서버 시작] ───────→ 포트 80에서 대기
  ├─→ GET / → HTML 페이지 반환
  ├─→ GET /status → {"state": true/false} 반환
  └─→ POST /led → LED 제어 + MQTT esp32/led에 "on"/"off" 발행
  ↓
[메인 루프] ────────────→ 1초마다 딜레이 (이벤트 기반 처리)
```

### 7.1 LED 제어 경로

**경로 1: HTTP 웹 브라우저**
```
브라우저 → POST /led {state:true} → led_post_handler()
  ├─→ GPIO 제어 (LED 켜기)
  ├─→ MQTT esp32/led에 "on" 발행
  └─→ 응답 {"state":true, "success":true}
```

**경로 2: MQTT 메시지**
```
MQTT 브로커 → esp32/led "on" → mqtt_event_handler()
  ├─→ GPIO 제어 (LED 켜기)
  └─→ esp32/status에 {"led":true} 발행
```

**경로 3: ESP32 자체 웹 페이지**
```
브라우저 → GET / → root_get_handler() → HTML 페이지
  └─→ JavaScript에서 POST /led 호출 → 경로 1과 동일
```

### 7.2 상태 동기화 메커니즘

1. **LED 상태 변경 시**:
   - HTTP 제어: `led_state` 업데이트 → GPIO 제어 → MQTT `esp32/led` 발행
   - MQTT 제어: MQTT 수신 → GPIO 제어 → `led_state` 업데이트 → MQTT `esp32/status` 발행

2. **상태 조회**:
   - HTTP: `GET /status` → `led_state` 반환
   - MQTT: `esp32/status` 구독 → LED 상태 JSON 수신

---

## 📝 코드 개선 제안

1. **WiFi 자격 증명 관리**
   - 하드코딩 대신 menuconfig 또는 NVS 사용
   - 예: `idf.py menuconfig`에서 설정 또는 런타임에 NVS에서 읽기

2. **JSON 파싱**
   - 현재 `strstr()` 기반 단순 파싱 → `cJSON` 라이브러리 사용 권장
   - 예: `cJSON_Parse()`, `cJSON_GetObjectItem()`

3. **에러 처리 강화**
   - MQTT 연결 실패 시 재연결 로직 추가
   - HTTP 서버 시작 실패 시 재시도

4. **보안**
   - 운영 환경에서는 MQTT 인증 추가(Username/Password)
   - HTTPS 사용 고려

5. **상태 동기화**
   - `publish_led_status()` 함수를 활용하여 코드 중복 제거

---

이 문서는 `blink_example_main.c`의 모든 기능을 상세히 분석한 것입니다. 각 함수와 이벤트 핸들러의 역할을 이해하면 ESP32에서 WiFi, HTTP, MQTT를 활용한 IoT 프로젝트를 확장하는 데 도움이 됩니다.


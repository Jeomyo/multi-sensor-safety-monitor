# 모듈화 가이드

이 프로젝트는 원래 단일 파일(`blink_example_main.c`)로 작성되었던 코드를 기능별 모듈로 분리하여 재사용성과 유지보수성을 향상시켰습니다.

## 📁 디렉토리 구조

```
main/
├── include/                    # 헤더 파일 디렉토리
│   ├── config.h               # 설정 상수 정의
│   ├── led_control.h          # LED 제어 모듈 헤더
│   ├── wifi_manager.h         # WiFi 관리 모듈 헤더
│   ├── mqtt_manager.h         # MQTT 관리 모듈 헤더
│   └── web_server.h           # HTTP 서버 모듈 헤더
├── blink_example_main.c       # 메인 파일 (간소화됨)
├── led_control.c              # LED 제어 모듈 구현
├── wifi_manager.c             # WiFi 관리 모듈 구현
├── mqtt_manager.c             # MQTT 관리 모듈 구현
├── web_server.c               # HTTP 서버 모듈 구현
└── CMakeLists.txt             # 빌드 설정
```

## 🔧 모듈별 설명

### 1. config.h - 설정 파일
**역할**: 모든 설정 상수를 한 곳에서 관리

```c
// WiFi, MQTT, LED, HTTP 서버 설정을 모두 포함
#define WIFI_SSID      "KT_GiGA_2371"
#define MQTT_BROKER_URL "mqtt://broker.hivemq.com"
#define LED_GPIO_PIN   2
```

**장점**:
- 설정 변경이 용이
- 환경별 설정 파일 분리 가능

### 2. led_control.c/h - LED 제어 모듈
**역할**: GPIO를 통한 LED 제어 기능 캡슐화

**주요 함수**:
- `led_control_init()`: LED 초기화
- `led_control_on()`: LED 켜기
- `led_control_off()`: LED 끄기
- `led_control_toggle()`: LED 토글
- `led_control_get_state()`: 현재 상태 조회

**장점**:
- LED 제어 로직을 독립적으로 관리
- 다른 프로젝트에서 재사용 가능
- 테스트가 용이

### 3. wifi_manager.c/h - WiFi 관리 모듈
**역할**: WiFi 연결 및 상태 관리

**주요 함수**:
- `wifi_manager_init()`: WiFi 초기화 및 연결 (블로킹)
- `wifi_manager_is_connected()`: 연결 상태 확인

**특징**:
- 재연결 로직 내장
- 이벤트 기반 연결 관리
- 연결 상태 추적

### 4. mqtt_manager.c/h - MQTT 관리 모듈
**역할**: MQTT 클라이언트 관리 및 메시지 처리

**주요 함수**:
- `mqtt_manager_init()`: MQTT 클라이언트 초기화
- `mqtt_manager_is_connected()`: 연결 상태 확인
- `mqtt_manager_publish()`: 메시지 발행
- `mqtt_manager_subscribe()`: 토픽 구독
- `mqtt_manager_register_led_callback()`: LED 제어 콜백 등록

**특징**:
- 콜백 기반 메시지 처리
- LED 제어 메시지를 자동 처리
- 상태 발행 기능 내장

### 5. web_server.c/h - HTTP 서버 모듈
**역할**: HTTP 웹서버 및 REST API 제공

**주요 함수**:
- `web_server_init()`: 서버 시작
- `web_server_deinit()`: 서버 중지

**제공 엔드포인트**:
- `GET /`: 메인 HTML 페이지
- `GET /status`: LED 상태 조회 (JSON)
- `POST /led`: LED 제어 (JSON)

**특징**:
- 내장 HTML 페이지 제공
- RESTful API 구조
- LED 제어 후 MQTT 상태 동기화

## 🔄 모듈 간 의존성

```
blink_example_main.c
    ├─→ led_control (독립적)
    ├─→ wifi_manager (독립적)
    ├─→ mqtt_manager
    │       └─→ config.h
    ├─→ web_server
    │       ├─→ led_control
    │       ├─→ mqtt_manager
    │       └─→ config.h
    └─→ config.h
```

**의존성 설명**:
- **led_control**: 완전 독립 모듈 (다른 모듈 의존 없음)
- **wifi_manager**: 완전 독립 모듈
- **mqtt_manager**: config.h만 의존
- **web_server**: led_control, mqtt_manager, config.h 의존
- **메인 파일**: 모든 모듈 통합

## 📝 사용 예제

### 새로운 프로젝트에서 LED 모듈만 사용하기

```c
#include "led_control.h"

void app_main(void)
{
    led_control_init();
    led_control_on();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    led_control_off();
}
```

### WiFi와 MQTT만 사용하기

```c
#include "wifi_manager.h"
#include "mqtt_manager.h"

void app_main(void)
{
    wifi_manager_init();
    mqtt_manager_init();
    
    // MQTT 메시지 발행
    mqtt_manager_publish("my/topic", "Hello", 5, 0, 0);
}
```

## 🚀 빌드 방법

모듈화 후에도 빌드 방법은 동일합니다:

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

CMakeLists.txt에 모든 소스 파일이 등록되어 있으므로 자동으로 빌드됩니다.

## 🔍 모듈화의 장점

### 1. 코드 재사용성
- 각 모듈을 독립적으로 다른 프로젝트에 사용 가능
- 예: LED 모듈만 별도 프로젝트에서 사용

### 2. 유지보수성
- 기능별로 파일이 분리되어 수정 범위 명확
- 버그 수정 시 해당 모듈만 수정하면 됨

### 3. 테스트 용이성
- 각 모듈을 독립적으로 테스트 가능
- 모의(mock) 객체 사용 가능

### 4. 가독성 향상
- 메인 파일이 간결해짐 (약 100줄 → 약 60줄)
- 각 모듈의 역할이 명확

### 5. 협업 효율성
- 여러 개발자가 동시에 다른 모듈 작업 가능
- Git 충돌 가능성 감소

## 🔄 기존 코드와의 비교

### 기존 (단일 파일)
- `blink_example_main.c`: 478줄
- 모든 기능이 한 파일에 집중
- 함수 간 의존성이 복잡

### 모듈화 후
- 메인 파일: 약 60줄
- 각 모듈: 50-150줄
- 명확한 책임 분리

## 📚 확장 예제

### 새로운 센서 모듈 추가

```c
// include/sensor_manager.h
esp_err_t sensor_manager_init(void);
float sensor_manager_read_temperature(void);
```

```c
// sensor_manager.c
// 온도 센서 읽기 구현
```

메인 파일에 통합:
```c
sensor_manager_init();
float temp = sensor_manager_read_temperature();
```

이렇게 하면 코드가 더욱 모듈화되고 확장 가능해집니다!


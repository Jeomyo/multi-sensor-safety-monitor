# ESP32 Firmware (ESP-IDF)

WiFi 연결, HTTP 웹서버, MQTT 연동으로 LED를 제어하고 상태를 발행하는 펌웨어 예제입니다. 브라우저에서 `/` 페이지로 접속해 LED를 켜고/끄고/토글할 수 있으며, MQTT 메시지(`esp32/led`)로도 제어 가능합니다.

## 주요 기능
- **WiFi STA 모드 연결**: SSID/비밀번호로 AP 접속, 재시도 로직 포함
- **HTTP 서버**:
  - `GET /` — 내장 UI 페이지 제공(LED 제어 버튼, 시스템 정보 표시)
  - `GET /status` — 현재 LED 상태 JSON 응답 `{ "state": true|false }`
  - `POST /led` — 본문 `{ "state": true|false }` 수신 시 LED 제어 및 MQTT 발행
- **MQTT 클라이언트**:
  - **브로커**: `mqtt://broker.hivemq.com:1883`
  - **구독**: `esp32/led` — `on`/`off` 수신 시 LED 제어
  - **발행**: `esp32/status` — 연결 시 "connected", 상태 변경 시 `{ "led": true|false }`
- **GPIO 제어**: 기본 GPIO 2번에 LED 연결, 상태에 따라 출력

## 빌드/플래시
```bash
idf.py set-target esp32s3   # 보드에 맞게 조정
idf.py menuconfig           # WiFi 등 설정 변경 시 권장(현재 코드는 상수 정의)
idf.py build
idf.py -p <PORT> flash monitor
```

## 설정 값(현재 코드)
- `WIFI_SSID`, `WIFI_PASS` — `blink_example_main.c` 상단 매크로로 고정 정의됨(실사용 시 menuconfig 전환 권장)
- MQTT: `MQTT_BROKER_URL`, `MQTT_BROKER_PORT`, `MQTT_CLIENT_ID`, `MQTT_TOPIC_LED`, `MQTT_TOPIC_STATUS`
- LED: `LED_GPIO_PIN`(기본 2)

## 파일 구조
- `blink_example_main.c` — 메인 애플리케이션(이벤트/서버/MQTT/LED 제어)
- `CMakeLists.txt` — 컴포넌트 의존성: `esp_wifi, esp_http_server, nvs_flash, mqtt, esp_driver_gpio, json`
- `Kconfig.projbuild` — 예제용 BLINK 설정 메뉴(현재 LED GPIO/주기 등 항목 포함)

## 동작 흐름(기능 설명)
- **웹 제어 ↔ MQTT 동기화**:
  - 사용자가 `/led` POST로 LED를 켜면, 장치는 실제 GPIO를 On/Off 후 `esp32/led`로 동일 동작을 발행합니다. 서버/다른 구독자가 상태를 인지할 수 있습니다.
  - MQTT의 `esp32/led`를 통해 `on`/`off` 수신 시에도 LED 상태를 갱신하고 `esp32/status`에 `{ "led": true|false }` JSON을 재발행합니다. 이를 통해 서버 UI와 장치 상태가 일관되게 유지됩니다.
- **상태 노출**:
  - `GET /status`는 단순하지만 상위 서버(예: `esp32-server`)가 폴링/프록시하여 UI에 반영할 수 있게 합니다.

## 보안/운영 팁
- WiFi 자격 증명은 코드 정적 상수 대신 menuconfig 또는 NVS/시크릿으로 관리 권장
- 공용 브로커(`broker.hivemq.com`)는 테스트용 — 운영 환경에서는 인증/사설 브로커 도입 권장
- `/led`의 JSON 파싱은 단순 파싱이므로 확장 시 `cJSON` 사용 확대 및 검증 강화 권장

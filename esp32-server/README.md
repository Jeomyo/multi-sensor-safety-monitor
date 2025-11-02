# ESP32 Server (Node.js)

ESP32 장치와 MQTT/웹소켓/DB를 연동하는 Node.js 백엔드입니다. REST API로 LED 제어, 임계치 설정, 이력/이벤트/센서 데이터 조회를 제공하고, 웹소켓으로 MQTT 수신 데이터를 실시간 브로드캐스트합니다.

## 주요 기능
- **MQTT 연동**: `broker.hivemq.com` 기본 브로커 사용, 토픽 `esp32/status` 구독, `esp32/led` 발행.
- **ESP32 HTTP 연동**: 장치의 `/status`, `/led` 엔드포인트와 통신.
- **실시간 대시보드**: `socket.io`로 MQTT 메시지를 브라우저에 전송, `public/index.html`에서 시각화.
- **DB 연동(MongoDB)**: LED 제어 이력, 센서 데이터, 시스템 이벤트, 사용자 활동 저장/조회.
- **임계치 관리**: 가스/온도/습도 임계치 조회·수정 및 대시보드 반영.

## 환경 변수
- `ESP32_IP` (기본 `172.30.1.92`)
- `MQTT_BROKER` (기본 `broker.hivemq.com`)
- `MQTT_PORT` (기본 `1883`)
- `MONGODB_URI` (기본 `mongodb://localhost:27017/esp32-iot`)
- `PORT` (기본 `3000`)

## MQTT 토픽
- **구독**: `esp32/status` — ESP32가 상태 JSON(예: `{ "led": true }`) 또는 문자열("connected") 발행
- **발행**: `esp32/led` — `on`/`off` 메시지로 LED 제어
- (선택) **센서**: `esp32/sensors` — `{ gas,temp,humi,motion }` 형태를 수신하면 대시보드 KPI/알람 반영

## REST API
- `GET /api/esp32/status` — ESP32 `/status` 프록시 조회
- `POST /api/esp32/led` — `{ state: true|false }`로 LED 제어
  - ESP32로 HTTP 전송 + MQTT `esp32/led` 발행 + DB 이력/활동 저장
- `POST /api/mqtt/publish` — `{ topic, message }` 임의 MQTT 발행
- `POST /api/alarms` — 경고/알람 이벤트 저장(경량)
- `GET /api/config` — 임계치/토픽 조회
- `PATCH /api/config` — 임계치 업데이트
- `GET /api/zones` / `PUT /api/zones` — 공간 트윈 ZONE 관리(메모리 기반)
- `GET /api/system/info` — 서버/ESP32/MQTT/DB 상태 요약
- `GET /api/history/led?limit=&deviceId=` — LED 제어 이력 조회
- `GET /api/sensors/data?limit=&deviceId=&sensorType=` — 센서 데이터 조회
- `GET /api/system/events?limit=&deviceId=&level=&from=&to=` — 시스템 이벤트 조회
- `GET /api/statistics?deviceId=&days=` — 최근 N일 통계(문서 수 집계)

## 데이터 모델 (Mongoose)
- `LedControl`: `deviceId, action('on'|'off'|'toggle'), state(bool), source('web'|'mqtt'|'api'|'mobile), ip, timestamp, metadata(userAgent,sessionId,requestId)`
- `SensorData`: `deviceId, sensorType('temperature'|'humidity'|'light'|'motion'|'distance'), value, unit, location, timestamp`
- `SystemEvent`: `deviceId, eventType('connection'|'disconnection'|'error'|'warning'|'info'), message, level('debug'|'info'|'warn'|'error'), metadata`
- `UserActivity`: `userId, sessionId, action, resource, ip, userAgent, duration(ms), success, timestamp`

## 대시보드(`public/index.html`)
- **가상 LED 제어**: 버튼 클릭 시 `POST /api/esp32/led` 호출, 실시간 상태 반영
- **시스템 정보**: `GET /api/system/info`로 MQTT/DB 상태, ESP32 URL 표시
- **KPI/알람**: `esp32/sensors` 메시지 기반 임계치 초과시 배너/강조 및 알람 저장
- **임계치 UI**: `GET/PATCH /api/config`로 가스/온도/습도 범위 갱신
- **ZONE 편집**: `GET/PUT /api/zones`로 공간 트윈 사각형 관리(드래그/리사이즈)
- **이벤트 표**: `GET /api/system/events`로 기간/레벨 필터 조회

## 실행 방법
```bash
cd esp32-server
npm install
# .env 파일에 ESP32_IP, MONGODB_URI 등 설정(선택)
npm start
```
서버가 시작되면 콘솔에 실제 포트와 MQTT/DB 연결 상태가 출력됩니다. 브라우저로 `http://localhost:3000` 접속.

## 구성 맥락(기능 설명)
- **LED 제어 연계**: 브라우저 → 서버(REST) → ESP32(HTTP) + MQTT 발행 → ESP32 상태 JSON 재발행 → 서버가 소켓으로 UI 반영. 이 흐름으로 UI/장치 상태 동기화와 이력/활동 로깅을 동시에 달성합니다.
- **임계치 기반 경보**: `esp32/sensors` 데이터에서 가스/온도 초과시 배너 표출과 함께 `POST /api/alarms`로 경고 저장. 운영 중 이상징후를 빠르게 감지·기록하기 위한 기능입니다.
- **ZONE 디지털 트윈**: 시설/공간을 SVG 사각형으로 표현하고 메모리에 저장해 UI에서 드래그/리사이즈. 기본 구조 제공 후 실제 좌표/로직 확장 가능.

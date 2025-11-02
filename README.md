# ESP32 IoT 센서 제어 시스템

ESP32 기반 IoT 센서 제어 시스템으로, 온습도 센서 데이터를 수집하고 MQTT를 통해 실시간 전송하며, 웹서버와 Node.js 서버를 통해 제어 및 데이터 로깅을 제공하는 통합 시스템입니다.

## 📋 프로젝트 개요

이 프로젝트는 ESP32S3 개발 보드를 사용하여 실시간 센서 데이터 수집, MQTT 통신, 웹 기반 제어, 그리고 데이터 영구 저장 기능을 제공하는 완전한 IoT 시스템입니다.

**주요 기술 스택:**
- **ESP32 펌웨어**: ESP-IDF (C 언어)
- **통신 프로토콜**: MQTT, HTTP
- **백엔드 서버**: Node.js (Express.js)
- **데이터베이스**: MongoDB
- **MQTT Broker**: HiveMQ (broker.hivemq.com)

---

## ✨ 주요 기능

### 1. 센서 하드웨어 지원
- ✅ **DHT11 온습도 센서** (GPIO 7)
  - 실시간 온도/습도 읽기 (2초 주기)
  - 데이터 검증 및 재시도 로직
  - 체크섬 검증 포함
- ✅ **RGB LED** (GPIO 4, 5, 6)
  - 빨강/초록/파랑 색상 개별 제어
  - 웹서버 및 API를 통한 원격 제어
  - 공통 음극 타입 지원
- ✅ **부저** (GPIO 8)
  - ON/OFF 제어
  - 웹서버 및 API를 통한 원격 제어

### 2. 실시간 통신
- ✅ **MQTT 데이터 발행**
  - 토픽: `esp32/sensor` (센서 데이터)
  - 토픽: `esp32/status` (시스템 상태)
  - 2초마다 자동 발행
  - JSON 형식 데이터 전송
- ✅ **HTTP 웹서버** (ESP32 내장)
  - 실시간 센서 데이터 표시
  - RGB LED 및 부저 원격 제어
  - RESTful API 제공

### 3. 데이터 관리
- ✅ **Node.js 서버**
  - MQTT 메시지 구독 및 저장
  - MongoDB 데이터 영구 저장
  - RESTful API 제공 (데이터 조회, 통계)
- ✅ **실시간 평균 계산**
  - 최근 5분간 데이터 버퍼링
  - 실시간 평균 온도/습도 계산

---

## 🏗️ 시스템 아키텍처

```
┌─────────────┐
│    ESP32    │
│  ┌────────┐ │
│  │ DHT11  │─┼──→ 센서 읽기 (2초)
│  └────────┘ │
│      │      │
│      ▼      │
│  ┌────────┐ │
│  │ MQTT   │─┼──→ esp32/sensor 발행
│  └────────┘ │
│      │      │
│      ▼      │
│  ┌────────┐ │
│  │ HTTP   │─┼──→ 웹서버 (포트 80)
│  └────────┘ │
└──────┬──────┘
       │
       ├─────────────┬─────────────────┐
       │             │                 │
       ▼             ▼                 ▼
┌──────────┐  ┌──────────┐    ┌──────────────┐
│  MQTT    │  │ 브라우저 │    │  라즈베리파이 │
│  Broker  │  │ (웹 UI)  │    │  QT 클라이언트│
└────┬─────┘  └──────────┘    └──────────────┘
     │
     ▼
┌──────────────┐
│ Node.js 서버 │
│  ┌─────────┐ │
│  │ MQTT    │─┼──→ 구독 및 수신
│  └─────────┘ │
│      │       │
│      ▼       │
│  ┌─────────┐ │
│  │ MongoDB │─┼──→ 데이터 저장
│  └─────────┘ │
│      │       │
│      ▼       │
│  ┌─────────┐ │
│  │ HTTP API│─┼──→ 로그 조회
│  └─────────┘ │
└──────────────┘
```

---

## 📦 하드웨어 구성

### 필수 하드웨어
- ESP32S3 개발 보드
- DHT11 온습도 센서 (10K 풀업 저항 포함)
- RGB LED (공통 음극)
- 부저 (3핀)
- WiFi 네트워크 접근 가능

### GPIO 핀 구성
```
DHT11      → GPIO 7
RGB LED R  → GPIO 4
RGB LED G  → GPIO 5
RGB LED B  → GPIO 6
부저       → GPIO 8
```

---

## 🚀 시작하기

### 1. ESP32 펌웨어 빌드 및 플래시

#### 사전 요구사항
- ESP-IDF v5.x 설치
- Python 3.x 설치
- USB 케이블 (ESP32와 PC 연결)

#### 빌드 및 플래시
```bash
# 프로젝트 디렉토리로 이동
cd project-mqtt

# 타겟 설정 (ESP32S3)
idf.py set-target esp32s3

# WiFi 및 MQTT 설정
# main/include/config.h 파일 수정:
#   - WIFI_SSID: WiFi 네트워크 이름
#   - WIFI_PASSWORD: WiFi 비밀번호
#   - MQTT_BROKER_URL: MQTT 브로커 주소 (기본: broker.hivemq.com)

# 빌드
idf.py build

# 플래시 및 모니터링
idf.py -p COM3 flash monitor
# (COM3는 실제 포트 번호로 변경)
```

### 2. Node.js 서버 설정

#### 사전 요구사항
- Node.js v18 이상
- MongoDB 설치 및 실행

#### 서버 실행
```bash
# Node.js 서버 디렉토리로 이동
cd esp32-server

# 의존성 설치
npm install

# 환경 변수 설정 (선택사항)
# .env 파일 생성 또는 환경 변수 설정:
#   ESP32_IP=172.30.1.63
#   MQTT_BROKER=broker.hivemq.com
#   MQTT_PORT=1883
#   MONGODB_URI=mongodb://localhost:27017/esp32-iot
#   PORT=3002

# 서버 시작
npm start
# 또는 Windows에서:
start_server.bat
```

### 3. 웹서버 접속

ESP32 웹서버에 접속하려면:
1. ESP32가 WiFi에 연결되면 시리얼 모니터에 IP 주소가 표시됩니다.
2. 브라우저에서 `http://[ESP32_IP]` 접속
   - 예: `http://172.30.1.63`

---

## 🌐 두 가지 웹서버

### ESP32 웹서버
- **URL**: `http://[ESP32_IP]` (예: `http://172.30.1.63`)
- **기능**: 실시간 센서 제어 및 모니터링
- **특징**: 
  - 실시간 온습도 표시 (2초마다 업데이트)
  - RGB LED 제어 (빨강/초록/파랑/끄기)
  - 부저 ON/OFF 제어
  - 5분 평균 온습도 표시
- **접근**: WiFi 네트워크 내 모든 기기에서 접근 가능

### Node.js 서버
- **URL**: `http://localhost:3002`
- **기능**: 데이터 로깅 및 관리 (개발자용)
- **특징**:
  - MongoDB에 모든 센서 데이터 영구 저장
  - RESTful API 제공
  - 통계 데이터 조회
- **접근**: PC에서만 (localhost)

자세한 비교는 [`SERVER_COMPARISON.md`](SERVER_COMPARISON.md) 참고

---

## 📡 MQTT 토픽 구조

### 발행 토픽 (ESP32 → Broker)

#### `esp32/sensor`
센서 데이터 (2초마다 발행)
```json
{
  "deviceId": "esp32-001",
  "temperature": 25.5,
  "humidity": 47.0,
  "timestamp": 1733245200,
  "valid": true
}
```

#### `esp32/status`
시스템 상태 (30초마다 발행)
```
Alive
```

### 구독 토픽 (선택사항)

향후 확장 가능:
- `esp32/led/control`: LED 제어 명령
- `esp32/buzzer/control`: 부저 제어 명령

---

## 🔌 RESTful API

### ESP32 웹서버 API

#### 센서 데이터 조회
```http
GET /sensor
```

**응답:**
```json
{
  "current": {
    "temperature": 25.5,
    "humidity": 47.0
  },
  "average_5min": {
    "temperature": 25.3,
    "humidity": 47.0
  }
}
```

#### RGB LED 제어
```http
POST /rgb_led
Content-Type: application/json

{
  "color": "red"  // "red", "green", "blue", "off"
}
```

또는:
```json
{
  "r": 255,
  "g": 0,
  "b": 0
}
```

#### 부저 제어
```http
POST /buzzer
Content-Type: application/json

{
  "state": true  // true: ON, false: OFF
}
```

### Node.js 서버 API

#### 센서 데이터 조회
```http
GET /api/sensors/data?limit=100&from=2024-01-01&to=2024-01-31
```

#### 통계 데이터 조회
```http
GET /api/sensors/statistics?days=7
```

**응답:**
```json
{
  "temperature": {
    "min": 20.5,
    "max": 30.2,
    "avg": 25.3
  },
  "humidity": {
    "min": 40.0,
    "max": 60.0,
    "avg": 47.0
  }
}
```

#### 시스템 정보
```http
GET /api/system/info
```

자세한 API 문서는 [`ARCHITECTURE_FLOW.md`](ARCHITECTURE_FLOW.md) 참고

---

## 📁 프로젝트 구조

```
project-mqtt/
├── main/                    # ESP32 펌웨어 소스 코드
│   ├── app_main.c          # 메인 애플리케이션
│   ├── web_server.c        # HTTP 웹서버
│   ├── mqtt_manager.c      # MQTT 클라이언트
│   ├── wifi_manager.c      # WiFi 관리
│   ├── dht11_sensor.c      # DHT11 센서 드라이버
│   ├── rgb_led_control.c   # RGB LED 제어
│   ├── buzzer_control.c    # 부저 제어
│   ├── sensor_average.c    # 평균 계산
│   └── include/            # 헤더 파일
├── esp32-server/           # Node.js 서버
│   ├── server.js           # 메인 서버 파일
│   ├── models/             # 데이터 모델
│   ├── services/           # 서비스 레이어
│   │   └── database.js     # MongoDB 서비스
│   └── public/             # 정적 파일
├── ARCHITECTURE_FLOW.md    # 아키텍처 문서
├── PROGRESS_REPORT.md      # 중간 보고서
├── TEST_GUIDE.md           # 테스트 가이드
└── README.md               # 이 파일
```

---

## 📚 문서

- [`ARCHITECTURE_FLOW.md`](ARCHITECTURE_FLOW.md) - 전체 시스템 아키텍처 및 데이터 흐름
- [`PROGRESS_REPORT.md`](PROGRESS_REPORT.md) - 중간 개발 보고서
- [`TEST_GUIDE.md`](TEST_GUIDE.md) - 테스트 가이드
- [`SERVER_COMPARISON.md`](SERVER_COMPARISON.md) - 두 웹서버 비교
- [`CAPTURE_CHECKLIST.md`](CAPTURE_CHECKLIST.md) - 캡처 체크리스트
- [`VIEW_LOGS.md`](VIEW_LOGS.md) - 로그 확인 방법

---

## 🧪 테스트

### ESP32 테스트
```bash
# 빌드 및 플래시 후 시리얼 모니터 확인
idf.py -p COM3 monitor
```

예상 출력:
```
I (xxxx) main: ✅ 모든 모듈 초기화 완료!
I (xxxx) main: 🌐 브라우저에서 접속: http://172.30.1.63
I (xxxx) main: 📡 MQTT 브로커: broker.hivemq.com:1883
I (xxxx) main: 🌡️  온도: 25.4°C, 💧 습도: 47%
```

### Node.js 서버 테스트
```bash
# 서버 실행 후 브라우저에서 확인
# http://localhost:3002/api/system/info
```

자세한 테스트 가이드는 [`TEST_GUIDE.md`](TEST_GUIDE.md) 참고

---

## 🔧 문제 해결

### DHT11 센서 오류
- **체크섬 오류**: 센서 핀 연결 확인, 전원 공급 확인
- **타임아웃**: 센서 핀을 올바른 GPIO에 연결했는지 확인
- **해결**: 재시도 로직이 내장되어 있으므로 일시적 오류는 자동 복구됩니다.

### WiFi 연결 실패
- `main/include/config.h`에서 WiFi SSID/비밀번호 확인
- WiFi 신호 강도 확인

### MQTT 연결 실패
- 인터넷 연결 확인
- MQTT 브로커 주소 확인 (`config.h`)
- 방화벽 설정 확인

### Node.js 서버 오류
- MongoDB 실행 상태 확인
- 포트 3002 사용 가능 여부 확인
- `.env` 파일 설정 확인

---

## 🛠️ 개발 환경

- **ESP-IDF**: v5.x
- **Node.js**: v18.16.1 이상
- **MongoDB**: 최신 버전
- **플랫폼**: Windows/Linux/macOS

- ## 참고자료
<img width="567" height="383" alt="image" src="https://github.com/user-attachments/assets/912915aa-44a8-42b8-a66c-3fb30dd48729" />
<img width="567" height="382" alt="image" src="https://github.com/user-attachments/assets/d0f82047-6a47-4daf-bbe8-875f46b06b73" />
<img width="567" height="657" alt="image" src="https://github.com/user-attachments/assets/7a3bd396-9ea0-4680-8f7d-130074893e4a" />

- 

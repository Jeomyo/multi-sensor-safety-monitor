# 시스템 통합 테스트 가이드

## 📋 테스트 전 준비사항

### 1. 필수 소프트웨어
- [ ] ESP-IDF 환경 설정 완료
- [ ] Node.js (v14 이상) 설치
- [ ] MongoDB 설치 및 실행
- [ ] MQTT 클라이언트 도구 (MQTT Explorer, MQTT.fx 등)

### 2. 네트워크 설정
- [ ] ESP32와 PC가 같은 WiFi 네트워크에 연결
- [ ] MQTT 브로커 접근 가능 (broker.hivemq.com)
- [ ] MongoDB가 실행 중

---

## 🔧 단계별 테스트

### Phase 1: ESP32 빌드 및 플래시

#### 1.1 코드 빌드
```bash
cd C:\Users\home2\project-mqtt
idf.py build
```

**예상 결과**: 빌드 성공 메시지

#### 1.2 ESP32 플래시
```bash
# COM 포트 확인 후 (예: COM3)
idf.py -p COM3 flash
```

**예상 결과**: 
```
Hash of data verified.
Leaving...
Hard resetting via RTS pin...
```

#### 1.3 시리얼 모니터 실행
```bash
idf.py -p COM3 monitor
```

**예상 로그**:
```
I (1234) main: ESP32 WiFi, WebServer & MQTT 시작 (모듈화 버전)
I (1235) wifi_manager: WiFi 초기화 시작
I (2345) wifi_manager: IP 할당됨: 192.168.1.100
I (2456) mqtt_manager: MQTT 브로커 연결됨
I (2457) mqtt_manager: 토픽 구독: esp32/led (msg_id=1)
I (2600) main: ✅ 모든 모듈 초기화 완료!
I (2601) main: 🌐 브라우저에서 접속: http://192.168.1.100
I (2602) main: 📨 MQTT 토픽: esp32/sensor (센서 데이터 발행)
```

**확인 사항**:
- [ ] WiFi 연결 성공
- [ ] IP 주소 할당됨
- [ ] MQTT 브로커 연결됨
- [ ] 웹서버 시작됨

#### 1.4 센서 데이터 발행 확인
**예상 로그 (2초마다)**:
```
I (5000) main: 🌡️  온도: 25.5°C, 💧 습도: 60.2%
I (5001) main: 📡 MQTT 센서 데이터 발행: {"deviceId":"esp32-001","temperature":25.5,"humidity":60.2,"timestamp":1704067200,"valid":true}
```

**확인 사항**:
- [ ] DHT11 센서 읽기 성공
- [ ] MQTT 메시지 발행됨
- [ ] JSON 형식 올바름

---

### Phase 2: MQTT 브로커 테스트

#### 2.1 MQTT Explorer로 테스트

**설정**:
- Broker: `broker.hivemq.com`
- Port: `1883`
- Protocol: `MQTT`

**구독할 토픽**:
- `esp32/sensor` - 센서 데이터 확인
- `esp32/status` - 상태 메시지 확인

**예상 결과**:
```
Topic: esp32/sensor
Message: {"deviceId":"esp32-001","temperature":25.5,"humidity":60.2,"timestamp":1704067200,"valid":true}
```

**확인 사항**:
- [ ] `esp32/sensor` 토픽에서 메시지 수신
- [ ] 2초마다 새로운 데이터 도착
- [ ] JSON 파싱 가능

#### 2.2 커맨드라인으로 테스트 (mosquitto_cli)

**Windows**:
```powershell
# mosquitto 클라이언트 설치 필요
mosquitto_sub -h broker.hivemq.com -p 1883 -t "esp32/sensor"
```

**Linux/Mac**:
```bash
mosquitto_sub -h broker.hivemq.com -p 1883 -t "esp32/sensor"
```

**예상 출력**:
```json
{"deviceId":"esp32-001","temperature":25.5,"humidity":60.2,"timestamp":1704067200,"valid":true}
{"deviceId":"esp32-001","temperature":25.6,"humidity":60.1,"timestamp":1704067202,"valid":true}
```

---

### Phase 3: Node.js 서버 설정 및 실행

#### 3.1 MongoDB 확인
```bash
# MongoDB 실행 확인
mongosh
# 또는
mongo
```

**MongoDB에서 확인**:
```javascript
use esp32-iot
db.sensordatas.find().limit(5)
```

#### 3.2 Node.js 서버 환경 설정

**esp32-server/.env 파일 생성**:
```env
MONGODB_URI=mongodb://localhost:27017/esp32-iot
MQTT_BROKER=broker.hivemq.com
MQTT_PORT=1883
ESP32_IP=192.168.1.100
PORT=3000
```

**참고**: `ESP32_IP`는 Phase 1에서 확인한 IP 주소로 변경

#### 3.3 의존성 설치
```bash
cd esp32-server
npm install
```

#### 3.4 서버 실행
```bash
npm start
```

**예상 로그**:
```
✅ MongoDB 연결 성공: mongodb://localhost:27017/esp32-iot
✅ MQTT 브로커에 연결됨: broker.hivemq.com
📡 MQTT 토픽 구독: esp32/status
📡 MQTT 토픽 구독: esp32/sensor
🚀 서버 시작: http://localhost:3000
```

**확인 사항**:
- [ ] MongoDB 연결 성공
- [ ] MQTT 브로커 연결 성공
- [ ] 토픽 구독 완료
- [ ] 서버 포트 3000에서 실행 중

#### 3.5 센서 데이터 수신 확인

**서버 콘솔 예상 로그**:
```
📨 MQTT 메시지 수신 [esp32/sensor]: {"deviceId":"esp32-001","temperature":25.5,"humidity":60.2,"timestamp":1704067200,"valid":true}
📊 센서 데이터 저장 완료: { deviceId: 'esp32-001', temperature: 25.5, humidity: 60.2 }
📊 센서 데이터 저장됨: 507f1f77bcf86cd799439011
```

**확인 사항**:
- [ ] `esp32/sensor` 토픽에서 메시지 수신
- [ ] MongoDB에 데이터 저장됨
- [ ] 온도/습도 데이터 모두 저장됨

---

### Phase 4: MongoDB 데이터 확인

#### 4.1 MongoDB Shell에서 확인

```bash
mongosh
```

```javascript
// 데이터베이스 선택
use esp32-iot

// 센서 데이터 확인
db.sensordatas.find().sort({timestamp: -1}).limit(10).pretty()

// 온도 데이터만
db.sensordatas.find({sensorType: "temperature"}).sort({timestamp: -1}).limit(10)

// 습도 데이터만
db.sensordatas.find({sensorType: "humidity"}).sort({timestamp: -1}).limit(10)

// 데이터 개수 확인
db.sensordatas.countDocuments()
```

**예상 결과**:
```json
{
  "_id": ObjectId("507f1f77bcf86cd799439011"),
  "deviceId": "esp32-001",
  "sensorType": "temperature",
  "value": 25.5,
  "unit": "celsius",
  "location": "room1",
  "timestamp": ISODate("2024-01-01T12:00:00.000Z")
}
```

**확인 사항**:
- [ ] 데이터가 저장되어 있음
- [ ] deviceId, sensorType, value, timestamp 필드 존재
- [ ] 온도/습도 데이터 모두 저장됨

---

### Phase 5: HTTP API 테스트

#### 5.1 센서 데이터 조회 API

**브라우저 또는 curl**:
```bash
# 전체 데이터 조회
curl http://localhost:3000/api/sensors/data?limit=10

# 온도 데이터만
curl http://localhost:3000/api/sensors/data?sensorType=temperature&limit=10

# 특정 장치
curl http://localhost:3000/api/sensors/data?deviceId=esp32-001&limit=10
```

**예상 응답**:
```json
{
  "success": true,
  "data": [
    {
      "_id": "507f1f77bcf86cd799439011",
      "deviceId": "esp32-001",
      "sensorType": "temperature",
      "value": 25.5,
      "unit": "celsius",
      "location": "room1",
      "timestamp": "2024-01-01T12:00:00.000Z"
    },
    ...
  ],
  "count": 10,
  "timestamp": "2024-01-01T12:05:00.000Z"
}
```

#### 5.2 센서 통계 API

```bash
curl http://localhost:3000/api/sensors/statistics?deviceId=esp32-001&days=7
```

**예상 응답**:
```json
{
  "success": true,
  "statistics": {
    "temperature": {
      "min": 20.0,
      "max": 30.0,
      "avg": 25.5,
      "count": 150
    },
    "humidity": {
      "min": 40.0,
      "max": 80.0,
      "avg": 60.2,
      "count": 150
    },
    "period": {
      "from": "2024-01-01T00:00:00.000Z",
      "to": "2024-01-08T00:00:00.000Z",
      "days": 7
    }
  },
  "timestamp": "2024-01-08T12:00:00.000Z"
}
```

#### 5.3 시스템 정보 API

```bash
curl http://localhost:3000/api/system/info
```

**확인 사항**:
- [ ] API 응답 성공 (200 OK)
- [ ] 데이터 형식 올바름
- [ ] MongoDB 연결 상태 확인 가능

---

### Phase 6: 웹 대시보드 확인

#### 6.1 브라우저 접속

```
http://localhost:3000
```

**확인 사항**:
- [ ] 대시보드 페이지 로드됨
- [ ] MQTT 연결 상태 표시
- [ ] MongoDB 연결 상태 표시

#### 6.2 WebSocket 실시간 데이터 확인

**브라우저 개발자 도구 콘솔**:
```javascript
// WebSocket 연결 확인
// (자동으로 연결되어 있어야 함)
```

**확인 사항**:
- [ ] WebSocket 연결됨
- [ ] `sensor-data` 이벤트 수신
- [ ] 실시간 데이터 업데이트

---

### Phase 7: ESP32 웹서버 테스트

#### 7.1 웹서버 접속

**브라우저**:
```
http://192.168.1.100
```

**확인 사항**:
- [ ] 웹페이지 로드됨
- [ ] 실시간 온습도 표시
- [ ] 5분 평균 표시
- [ ] RGB LED 제어 버튼 작동
- [ ] 부저 제어 버튼 작동

#### 7.2 센서 데이터 API 테스트

```bash
curl http://192.168.1.100/sensor
```

**예상 응답**:
```json
{
  "current": {
    "temperature": 25.5,
    "humidity": 60.2
  },
  "average_5min": {
    "temperature": 25.3,
    "humidity": 60.5
  }
}
```

---

## 🐛 트러블슈팅

### 문제 1: ESP32에서 MQTT 연결 실패

**증상**: 
```
E (xxxx) mqtt_manager: MQTT 브로커 연결 실패
```

**해결 방법**:
1. WiFi 연결 확인
2. MQTT 브로커 주소 확인 (`broker.hivemq.com`)
3. 방화벽 설정 확인

### 문제 2: 센서 데이터가 MongoDB에 저장되지 않음

**증상**: 
- MQTT 메시지는 수신되지만 저장 안 됨

**해결 방법**:
1. MongoDB 실행 확인
   ```bash
   mongosh
   ```
2. MongoDB URI 확인 (`.env` 파일)
3. Node.js 서버 로그 확인
4. 데이터베이스 권한 확인

### 문제 3: DHT11 읽기 실패

**증상**:
```
W (xxxx) main: DHT11 읽기 실패 또는 데이터 무효
```

**해결 방법**:
1. 센서 핀 연결 확인 (GPIO 7)
2. 전원 연결 확인 (VCC, GND)
3. 풀업 저항 확인 (10K)
4. 센서 지연 시간 증가

### 문제 4: Node.js 서버가 시작되지 않음

**증상**:
```
Error: Cannot find module 'xxx'
```

**해결 방법**:
```bash
cd esp32-server
rm -rf node_modules
npm install
```

### 문제 5: MQTT 메시지가 수신되지 않음

**증상**:
- ESP32에서는 발행하는데 서버에서 수신 안 됨

**해결 방법**:
1. MQTT 브로커 연결 확인
2. 토픽 이름 확인 (`esp32/sensor`)
3. MQTT Explorer로 직접 확인
4. 네트워크 연결 확인

---

## ✅ 테스트 체크리스트

### ESP32
- [ ] WiFi 연결 성공
- [ ] MQTT 브로커 연결 성공
- [ ] 웹서버 시작됨
- [ ] DHT11 센서 읽기 성공
- [ ] MQTT 메시지 발행됨

### MQTT
- [ ] 브로커 연결 가능
- [ ] `esp32/sensor` 토픽 구독 가능
- [ ] 메시지 수신 확인

### Node.js 서버
- [ ] MongoDB 연결 성공
- [ ] MQTT 브로커 연결 성공
- [ ] 토픽 구독 완료
- [ ] 센서 데이터 수신됨
- [ ] MongoDB에 저장됨

### MongoDB
- [ ] 데이터베이스 실행 중
- [ ] 데이터 저장됨
- [ ] 쿼리 가능

### HTTP API
- [ ] `/api/sensors/data` 작동
- [ ] `/api/sensors/statistics` 작동
- [ ] `/api/system/info` 작동

### 웹 인터페이스
- [ ] ESP32 웹서버 접속 가능
- [ ] Node.js 대시보드 접속 가능
- [ ] 실시간 데이터 표시

---

## 📊 성능 테스트

### 데이터 발행 속도
- **목표**: 2초마다 1개 메시지
- **측정**: 1분간 메시지 개수 (예상: 30개)

### 데이터 저장 속도
- **목표**: MQTT 수신 후 100ms 이내 MongoDB 저장
- **측정**: 서버 로그 타임스탬프 확인

### API 응답 속도
- **목표**: 100개 데이터 조회 시 500ms 이내
- **측정**: `curl -w "%{time_total}" http://localhost:3000/api/sensors/data?limit=100`

---

## 🎯 다음 단계

테스트 성공 후:
1. ✅ 라즈베리파이 QT 클라이언트 개발
2. ✅ 데이터 시각화 (Grafana 등)
3. ✅ 알람 기능 추가
4. ✅ 다중 ESP32 지원

---

## 📝 테스트 결과 기록

### 테스트 환경
- **날짜**: 
- **ESP32 IP**: 
- **Node.js 서버**: 
- **MongoDB 버전**: 
- **MQTT 브로커**: broker.hivemq.com

### 테스트 결과
- **ESP32 빌드**: ✅ / ❌
- **MQTT 발행**: ✅ / ❌
- **MongoDB 저장**: ✅ / ❌
- **HTTP API**: ✅ / ❌

### 발견된 이슈
1. 
2. 
3. 


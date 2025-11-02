# 빠른 테스트 가이드

## 현재 상태 확인

### ✅ 확인 완료
- Node.js v18.16.1 설치됨
- .env 파일 존재 (포트: 3002, ESP32_IP: 172.30.1.92)
- esp32-server 디렉토리 존재

### ⚠️ 확인 필요
- MongoDB 실행 상태
- ESP32 연결 상태

---

## 테스트 순서

### 1. MongoDB 실행 확인
```powershell
# MongoDB 서비스 확인
Get-Service -Name MongoDB* -ErrorAction SilentlyContinue

# 또는 프로세스 확인
Get-Process -Name mongod -ErrorAction SilentlyContinue
```

MongoDB가 실행되지 않았다면:
- Windows 서비스에서 MongoDB 시작
- 또는 `mongod` 명령으로 수동 실행

### 2. Node.js 서버 실행
```powershell
cd C:\Users\home2\project-mqtt\esp32-server
npm start
```

**예상 로그**:
```
✅ MongoDB 연결 성공: mongodb://localhost:27017/esp32-iot
✅ MQTT 브로커에 연결됨: broker.hivemq.com
📡 MQTT 토픽 구독: esp32/status
📡 MQTT 토픽 구독: esp32/sensor
🚀 서버 시작: http://localhost:3002
```

### 3. ESP32 시리얼 모니터 확인

**ESP-IDF 환경에서 실행**:
```bash
# ESP-IDF 환경 활성화 후
idf.py -p COM3 monitor
```

**확인할 로그**:
```
I (xxxx) main: 📡 MQTT 센서 데이터 발행: {"deviceId":"esp32-001","temperature":25.5,"humidity":60.2,"timestamp":1704067200,"valid":true}
```

### 4. MQTT Explorer로 확인

**설정**:
- Host: `broker.hivemq.com`
- Port: `1883`
- Topic 구독: `esp32/sensor`

**확인**: JSON 메시지가 2초마다 수신되는지

### 5. Node.js 서버 로그 확인

**확인할 로그**:
```
📨 MQTT 메시지 수신 [esp32/sensor]: {...}
📊 센서 데이터 저장 완료: { deviceId: 'esp32-001', temperature: 25.5, humidity: 60.2 }
📊 센서 데이터 저장됨: 507f1f77bcf86cd799439011
```

### 6. HTTP API 테스트

**새 터미널에서**:
```powershell
# 센서 데이터 조회
curl http://localhost:3002/api/sensors/data?limit=10

# 통계 조회
curl http://localhost:3002/api/sensors/statistics?days=7
```

### 7. MongoDB 데이터 확인

MongoDB가 실행 중이라면:
```javascript
// MongoDB Shell 또는 MongoDB Compass에서
use esp32-iot
db.sensordatas.find().sort({timestamp: -1}).limit(5).pretty()
```

---

## 문제 해결

### MongoDB 연결 실패
- MongoDB 서비스 시작
- `mongodb://localhost:27017` 연결 확인

### MQTT 메시지 수신 안 됨
- ESP32가 실행 중인지 확인
- MQTT Explorer로 직접 확인
- 토픽 이름 확인 (`esp32/sensor`)

### 데이터 저장 안 됨
- MongoDB 연결 확인
- Node.js 서버 로그 확인
- 데이터베이스 권한 확인


# 📸 캡처 체크리스트

보고서 작성을 위해 다음 화면들을 캡처해주세요.

---

## 1️⃣ ESP32 시리얼 모니터 캡처

### 필수 캡처 (3개)

#### 1-1. 초기화 완료 로그
**위치**: ESP-IDF 시리얼 모니터  
**캡처 내용**:
```
I (xxxx) main: ✅ 모든 모듈 초기화 완료!
I (xxxx) main: 🌐 브라우저에서 접속: http://192.168.x.x
I (xxxx) main: 📡 MQTT 브로커: broker.hivemq.com:1883
I (xxxx) main: 📨 MQTT 토픽: esp32/sensor (센서 데이터 발행)
```

#### 1-2. 센서 데이터 읽기 로그
**캡처 내용**:
```
I (xxxx) main: 🌡️  온도: 25.4°C, 💧 습도: 47%
```

#### 1-3. MQTT 발행 로그
**캡처 내용**:
```
I (xxxx) main: 📡 MQTT 센서 데이터 발행: {"deviceId":"esp32-001","temperature":25.4,"humidity":47.0,"timestamp":xxx,"valid":true}
```

---

## 2️⃣ Node.js 서버 로그 캡처

### 필수 캡처 (3개)

#### 2-1. 서버 시작 로그
**위치**: PowerShell/터미널 (서버 실행 창)  
**캡처 내용**:
```
✅ MongoDB 연결 성공: mongodb://localhost:27017/esp32-iot
✅ MQTT 브로커에 연결됨: broker.hivemq.com
📡 MQTT 토픽 구독: esp32/status
📡 MQTT 토픽 구독: esp32/sensor
🚀 서버 시작: http://localhost:3002
```

#### 2-2. 센서 데이터 수신 로그 (5-10개)
**캡처 내용**:
```
📨 MQTT 메시지 수신 [esp32/sensor]: {"deviceId":"esp32-001","temperature":25.4,"humidity":47.0,...}
📊 센서 데이터 저장됨: new ObjectId('...')
📊 센서 데이터 저장 완료: { deviceId: 'esp32-001', temperature: 25.4, humidity: 47 }
```

#### 2-3. 데이터 저장 연속 로그
**캡처 내용**: 여러 개의 저장 로그가 연속으로 표시되는 화면

---

## 3️⃣ MQTT Explorer 화면 캡처

### 필수 캡처 (2개)

#### 3-1. 연결 설정 화면
**캡처 내용**:
- Host: `broker.hivemq.com`
- Port: `1883`
- 연결 상태: Connected

#### 3-2. 토픽 구독 및 메시지 수신 화면
**캡처 내용**:
- 구독 토픽: `esp32/sensor`
- 수신된 메시지 목록 (최소 3개)
- JSON 메시지 내용

---

## 4️⃣ 웹서버 화면 캡처

### 필수 캡처 (2개)

#### 4-1. 메인 페이지 - 센서 데이터 표시
**URL**: `http://<ESP32_IP>`  
**캡처 내용**:
- 실시간 온도/습도 값이 표시되는 부분
- 5분 평균 값이 표시되는 부분
- 전체 페이지 레이아웃

#### 4-2. 제어 버튼 화면
**캡처 내용**:
- RGB LED 제어 버튼 (빨강/초록/파랑/끄기)
- 부저 제어 버튼 (ON/OFF)
- 버튼 클릭 후 상태 변경 확인 (선택)

---

## 5️⃣ HTTP API 응답 캡처

### 필수 캡처 (3개)

**실행 위치**: PowerShell

#### 5-1. 시스템 정보 API
```powershell
Invoke-WebRequest -Uri http://localhost:3002/api/system/info -UseBasicParsing | Select-Object -Property Content
```

**캡처 내용**: JSON 응답 (MongoDB 연결 상태, MQTT 연결 상태 포함)

#### 5-2. 센서 데이터 조회 API
```powershell
Invoke-WebRequest -Uri "http://localhost:3002/api/sensors/data?limit=5" -UseBasicParsing | Select-Object -Property Content
```

**캡처 내용**: JSON 응답 (저장된 센서 데이터 5개)

#### 5-3. 통계 API
```powershell
Invoke-WebRequest -Uri "http://localhost:3002/api/sensors/statistics?days=7" -UseBasicParsing | Select-Object -Property Content
```

**캡처 내용**: JSON 응답 (온도/습도 통계)

---

## 6️⃣ MongoDB 데이터 캡처 (선택사항)

### 선택 캡처 (1개)

**도구**: MongoDB Compass 또는 MongoDB Shell

#### 6-1. 저장된 데이터 확인
**캡처 내용**:
- 데이터베이스: `esp32-iot`
- 컬렉션: `sensordatas`
- 저장된 데이터 목록 (3-5개)
- 데이터 개수 표시

**또는 MongoDB Shell**:
```javascript
use esp32-iot
db.sensordatas.find().sort({timestamp: -1}).limit(5).pretty()
```

---

## 📋 캡처 우선순위

### 필수 (보고서에 반드시 포함)
1. ✅ ESP32 시리얼 모니터 - 초기화 완료 로그
2. ✅ ESP32 시리얼 모니터 - 센서 데이터 발행 로그
3. ✅ Node.js 서버 - 데이터 수신 및 저장 로그
4. ✅ 웹서버 - 메인 페이지 (센서 데이터 표시)
5. ✅ HTTP API - 시스템 정보 응답

### 권장 (보고서 품질 향상)
6. ✅ MQTT Explorer - 토픽 구독 화면
7. ✅ HTTP API - 센서 데이터 조회 응답
8. ✅ HTTP API - 통계 응답

### 선택 (추가 설명용)
9. ⭕ MongoDB - 저장된 데이터 확인
10. ⭕ 웹서버 - 제어 버튼 동작 화면

---

## 💡 캡처 팁

### 화면 캡처 시 주의사항
- **해상도**: 가능한 높은 해상도로 캡처
- **텍스트 가독성**: 로그 텍스트가 잘 보이도록 확대
- **프레임**: 중요 로그 부분만 잘라서 캡처 (너무 길지 않게)
- **타임스탬프**: 가능하면 타임스탬프가 포함된 로그 캡처

### 편집 팁
- **주석 추가**: 캡처 화면에 화살표나 박스로 중요 부분 표시
- **순서 번호**: 여러 캡처를 순서대로 번호 표시
- **설명 추가**: 각 캡처 하단에 간단한 설명 추가

---

## 📝 체크리스트

캡처 완료 후 체크:

- [ ] ESP32 시리얼 모니터 (3개)
- [ ] Node.js 서버 로그 (3개)
- [ ] MQTT Explorer (2개)
- [ ] 웹서버 화면 (2개)
- [ ] HTTP API 응답 (3개)
- [ ] MongoDB 데이터 (1개, 선택)

**총 최소**: 11개 캡처  
**권장**: 13개 캡처


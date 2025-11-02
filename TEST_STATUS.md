# 테스트 상태

## ✅ 완료된 항목

### 1. MongoDB 실행 확인
- ✅ MongoDB 서비스 실행 중 (PID: 4676)
- ✅ 포트: 27017

### 2. Node.js 서버 설정
- ✅ Node.js v18.16.1 설치됨
- ✅ .env 파일 확인됨 (포트: 3002)
- ✅ 서버 백그라운드 실행 중

### 3. 환경 설정
- ✅ ESP32_IP: 172.30.1.92
- ✅ MQTT_BROKER: broker.hivemq.com
- ✅ MONGODB_URI: mongodb://localhost:27017/esp32-iot

---

## 🔄 진행 중인 항목

### 4. Node.js 서버 연결 확인
- 서버 시작 대기 중...
- MongoDB 연결 확인 필요
- MQTT 브로커 연결 확인 필요

---

## ⏳ 대기 중인 항목

### 5. ESP32 테스트
- [ ] ESP32 시리얼 모니터 실행
- [ ] MQTT 메시지 발행 확인
- [ ] 웹서버 접속 확인

### 6. MQTT 메시지 확인
- [ ] MQTT Explorer 설치/실행
- [ ] esp32/sensor 토픽 구독
- [ ] 메시지 수신 확인

### 7. 데이터 저장 확인
- [ ] MongoDB 데이터 확인
- [ ] 데이터 개수 확인

### 8. HTTP API 테스트
- [ ] /api/sensors/data 테스트
- [ ] /api/sensors/statistics 테스트

---

## 📝 다음 단계

1. **Node.js 서버 로그 확인**
   - 서버 콘솔에서 MongoDB/MQTT 연결 상태 확인

2. **ESP32 실행**
   - ESP-IDF 환경에서 `idf.py -p COM3 monitor` 실행
   - 센서 데이터 발행 로그 확인

3. **MQTT Explorer**
   - broker.hivemq.com:1883 연결
   - esp32/sensor 토픽 구독

4. **API 테스트**
   ```powershell
   curl http://localhost:3002/api/sensors/data?limit=10
   curl http://localhost:3002/api/sensors/statistics?days=7
   ```


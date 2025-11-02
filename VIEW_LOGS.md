# Node.js 서버 로그 확인 방법

## 🔍 현재 상황

**서버 실행 중**: PID 16828, 포트 3002에서 실행 중

터미널에서 확인된 로그:
- ✅ MQTT 메시지 수신: `esp32/sensor` 토픽에서 데이터 수신 중
- ✅ MongoDB 저장: 센서 데이터 정상 저장
- ✅ 온도: 25.3-25.4°C, 습도: 47%

---

## 📺 로그 확인 방법

### 방법 1: 실행 중인 터미널 창 찾기

1. **작업 표시줄**에서 PowerShell/터미널 아이콘 확인
2. 여러 터미널이 열려있다면 각각 확인
3. Node.js 서버를 실행한 창에서 로그 확인

**찾는 팁**:
- 작업 표시줄에서 PowerShell 아이콘 위에 마우스 올리면 여러 창 미리보기
- 각 창 제목 확인: "Node.js", "server.js" 등이 있을 수 있음

### 방법 2: 서버 재시작 (로그 보면서) ⭐ 추천

**기존 서버 종료 후 재시작**:

```powershell
# 1. 기존 서버 종료
Stop-Process -Id 16828 -Force

# 2. 서버 실행 (로그가 바로 보임)
cd C:\Users\home2\project-mqtt\esp32-server
node server.js
```

**또는 배치 파일 사용**:
```powershell
cd C:\Users\home2\project-mqtt\esp32-server
.\start_server.bat
```

**장점**:
- ✅ 실시간 로그 확인
- ✅ Ctrl+C로 종료 가능
- ✅ 에러 발생 시 즉시 확인

### 방법 3: 로그 파일로 저장

```powershell
cd C:\Users\home2\project-mqtt\esp32-server
node server.js | Tee-Object -FilePath server.log
```

로그를 `server.log` 파일에도 저장하면서 화면에도 표시됩니다.

---

## 📊 확인할 수 있는 로그 내용

### 서버 시작 로그
```
✅ MongoDB 연결 성공: mongodb://localhost:27017/esp32-iot
✅ MQTT 브로커에 연결됨: broker.hivemq.com
📡 MQTT 토픽 구독: esp32/status
📡 MQTT 토픽 구독: esp32/sensor
🚀 서버 시작: http://localhost:3002
```

### 실시간 센서 데이터 (2초마다)
```
📨 MQTT 메시지 수신 [esp32/sensor]: {"deviceId":"esp32-001","temperature":25.4,"humidity":47.0,"timestamp":555,"valid":true}
📊 센서 데이터 저장됨: new ObjectId('69075d0960cf37c93c68934b')
📊 센서 데이터 저장됨: new ObjectId('69075d0960cf37c93c68934d')
📊 센서 데이터 저장 완료: { deviceId: 'esp32-001', temperature: 25.4, humidity: 47 }
```

### 상태 메시지 (주기적으로)
```
📨 MQTT 메시지 수신 [esp32/status]: Alive
```

---

## 🎯 빠른 확인

**현재 서버 상태 확인**:
```powershell
# 서버가 실행 중인지 확인
Get-NetTCPConnection -LocalPort 3002 | Select-Object -Property State, OwningProcess

# API 테스트
Invoke-WebRequest -Uri http://localhost:3002/api/system/info -UseBasicParsing
```

---

## 💡 추천 방법

**가장 쉬운 방법**: 새로운 터미널에서 서버 재시작

1. 새 PowerShell 창 열기
2. 다음 명령 실행:
   ```powershell
   cd C:\Users\home2\project-mqtt\esp32-server
   node server.js
   ```

이렇게 하면:
- ✅ 로그가 실시간으로 표시됨
- ✅ Ctrl+C로 종료 가능
- ✅ 에러 발생 시 즉시 확인 가능

---

## 📝 참고

- **서버 코드**: `esp32-server/server.js`
- **설정 파일**: `esp32-server/.env`
- **포트**: 3002
- **현재 실행 중인 프로세스**: PID 16828

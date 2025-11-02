# 센서 데이터 아키텍처 설계

## 목표
- **라즈베리파이 QT**: 실시간 센서 데이터 수신
- **ESP32 웹서버**: 데이터 로그/DB 역할 (개발자용)

## 추천 아키텍처: 하이브리드 방식

```
┌─────────────┐
│    ESP32    │
│  (센서 읽기) │
└──────┬──────┘
       │
       ├──────────────────────┐
       │                      │
       ▼                      ▼
┌─────────────┐      ┌──────────────┐
│    MQTT     │      │  HTTP API    │
│ (실시간 전송)│      │  (로그 저장) │
└──────┬──────┘      └──────┬───────┘
       │                    │
       │                    ▼
       │            ┌──────────────┐
       │            │ Node.js 서버 │
       │            │ (esp32-server)│
       │            │  MongoDB 저장 │
       │            └──────┬───────┘
       │                   │
       │                   ▼
       │            ┌──────────────┐
       │            │   HTTP API   │
       │            │ (로그 조회)  │
       │            └──────────────┘
       │
       ▼
┌─────────────┐
│라즈베리파이 QT│
│ (실시간 구독) │
└─────────────┘
```

## 구현 방식

### 1. ESP32에서 센서 데이터 MQTT 발행

**토픽**: `esp32/sensor`
**메시지 형식**:
```json
{
  "temperature": 25.5,
  "humidity": 60.2,
  "timestamp": 1234567890,
  "deviceId": "esp32-001"
}
```

### 2. 라즈베리파이 QT에서 MQTT 구독

```cpp
// Qt MQTT 클라이언트 예시
QMQTT::Client client;
client.setHostName("broker.hivemq.com");
client.setPort(1883);
client.connectToHost();

QMQTT::Message message(0, "esp32/sensor", QByteArray::fromRawData(jsonData, strlen(jsonData)));
client.subscribe("esp32/sensor", 0);

// 메시지 수신 처리
connect(&client, &QMQTT::Client::received, [](const QMQTT::Message &msg) {
    QJsonDocument doc = QJsonDocument::fromJson(msg.payload());
    QJsonObject obj = doc.object();
    float temp = obj["temperature"].toDouble();
    float humi = obj["humidity"].toDouble();
    // UI 업데이트
});
```

### 3. Node.js 서버에서 MQTT 구독 → DB 저장

**esp32-server/server.js** 수정:
- `esp32/sensor` 토픽 구독
- MongoDB에 센서 데이터 저장
- HTTP API로 로그 조회 제공

### 4. 개발자가 HTTP API로 로그 조회

```bash
# 최근 센서 데이터 조회
GET http://nodejs-server:3000/api/sensors/data?limit=100&deviceId=esp32-001

# 특정 기간 데이터 조회
GET http://nodejs-server:3000/api/sensors/data?from=2024-01-01&to=2024-01-31

# 통계 조회
GET http://nodejs-server:3000/api/sensors/statistics?deviceId=esp32-001&days=7
```

## 장점

1. **실시간성**: MQTT로 라즈베리파이 QT에 즉시 데이터 전달
2. **데이터 저장**: Node.js 서버에서 영구 저장 (MongoDB)
3. **확장성**: 여러 ESP32 장치 지원 가능
4. **분리**: ESP32는 센서 읽기만, 서버는 저장/조회만
5. **유연성**: 라즈베리파이는 실시간, 개발자는 히스토리 분석

## 구현 단계

### Phase 1: ESP32에서 MQTT 발행
- [ ] `app_main.c`에서 센서 읽기 시 MQTT 발행
- [ ] JSON 형식으로 데이터 포맷팅

### Phase 2: Node.js 서버 확장
- [ ] `esp32/sensor` 토픽 구독 추가
- [ ] MongoDB에 센서 데이터 저장 로직 추가
- [ ] HTTP API 엔드포인트 확장

### Phase 3: 라즈베리파이 QT 클라이언트
- [ ] MQTT 클라이언트 연결
- [ ] `esp32/sensor` 토픽 구독
- [ ] UI에 실시간 데이터 표시

### Phase 4: HTTP API 활용
- [ ] 개발자용 대시보드 구성
- [ ] 데이터 조회/분석 기능 추가


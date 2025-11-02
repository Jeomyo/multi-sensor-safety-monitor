# 센서 데이터 통합 아키텍처 - 전체 흐름 문서

## 📋 목차
1. [시스템 개요](#시스템-개요)
2. [아키텍처 다이어그램](#아키텍처-다이어그램)
3. [데이터 흐름](#데이터-흐름)
4. [구현 상세](#구현-상세)
5. [API 명세](#api-명세)
6. [설정 및 배포](#설정-및-배포)

---

## 시스템 개요

### 구성 요소
1. **ESP32**: 센서 데이터 수집 및 발행
2. **MQTT Broker**: 실시간 메시지 중계 (broker.hivemq.com)
3. **라즈베리파이 QT**: 실시간 센서 데이터 수신 및 UI 표시
4. **Node.js 서버**: 데이터 로깅 및 DB 관리
5. **MongoDB**: 센서 데이터 영구 저장

### 역할 분담
- **ESP32**: 센서 읽기 → MQTT 발행 + HTTP API (제어용)
- **라즈베리파이 QT**: MQTT 구독 → 실시간 데이터 표시
- **Node.js 서버**: MQTT 구독 → DB 저장 → HTTP API (로그 조회)

---

## 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32                                │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐ │
│  │ DHT11 Sensor │───▶│  센서 읽기    │───▶│  데이터 포맷  │ │
│  │ (온도/습도)   │    │  (2초마다)    │    │   (JSON)     │ │
│  └──────────────┘    └──────┬───────┘    └──────┬───────┘ │
│                             │                    │          │
│                             ▼                    ▼          │
│                    ┌────────────────────────────────────┐  │
│                    │       MQTT Client                 │  │
│                    │  (esp32/sensor 토픽 발행)         │  │
│                    └──────────────┬────────────────────┘  │
│                                   │                        │
└───────────────────────────────────┼────────────────────────┘
                                    │
                                    │ MQTT Message
                                    │ {"temperature": 25.5,
                                    │  "humidity": 60.2, ...}
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
        ┌──────────────────────┐      ┌──────────────────────┐
        │   MQTT Broker        │      │   MQTT Broker        │
        │ (broker.hivemq.com)  │      │ (broker.hivemq.com)  │
        └──────────┬───────────┘      └──────────┬───────────┘
                   │                             │
                   │ esp32/sensor                │ esp32/sensor
                   │ 구독                        │ 구독
                   │                             │
                   ▼                             ▼
    ┌─────────────────────────┐    ┌──────────────────────────┐
    │   라즈베리파이 QT        │    │    Node.js 서버           │
    │                         │    │   (esp32-server)         │
    │  ┌───────────────────┐  │    │                          │
    │  │ MQTT Client       │  │    │  ┌────────────────────┐  │
    │  │ (Qt MQTT)         │  │    │  │ MQTT Client        │  │
    │  └────────┬──────────┘  │    │  │ (mqtt.js)          │  │
    │           │             │    │  └─────────┬──────────┘  │
    │           ▼             │    │            │             │
    │  ┌───────────────────┐  │    │            ▼             │
    │  │ 데이터 파싱        │  │    │  ┌────────────────────┐  │
    │  │ (JSON)            │  │    │  │ 데이터 파싱        │  │
    │  └────────┬──────────┘  │    │  │ (JSON)             │  │
    │           │             │    │  └─────────┬──────────┘  │
    │           ▼             │    │            │             │
    │  ┌───────────────────┐  │    │            ▼             │
    │  │ UI 업데이트        │  │    │  ┌────────────────────┐  │
    │  │ (실시간 표시)      │  │    │  │ MongoDB 저장       │  │
    │  └───────────────────┘  │    │  │ (DatabaseService)  │  │
    │                         │    │  └─────────┬──────────┘  │
    └─────────────────────────┘    │            │             │
                                   │            ▼             │
                                   │  ┌────────────────────┐  │
                                   │  │ HTTP API 서버      │  │
                                   │  │ (Express.js)       │  │
                                   │  └─────────┬──────────┘  │
                                   │            │             │
                                   └────────────┼─────────────┘
                                                │
                                                │ HTTP GET
                                                │ /api/sensors/data
                                                │
                                   ┌────────────┴─────────────┐
                                   │                          │
                                   ▼                          ▼
                        ┌──────────────────┐    ┌──────────────────┐
                        │   MongoDB        │    │   개발자         │
                        │  (데이터 저장)    │    │  (로그 조회)     │
                        └──────────────────┘    └──────────────────┘
```

---

## 데이터 흐름

### 1. 센서 데이터 수집 (ESP32)

**주기**: 2초마다
**처리 흐름**:
```
DHT11 센서 읽기
    ↓
데이터 검증 (checksum)
    ↓
JSON 형식으로 변환
    ↓
MQTT 발행 (esp32/sensor 토픽)
```

**메시지 형식**:
```json
{
  "deviceId": "esp32-001",
  "temperature": 25.5,
  "humidity": 60.2,
  "timestamp": 1704067200,
  "valid": true
}
```

### 2. 실시간 데이터 수신 (라즈베리파이 QT)

**처리 흐름**:
```
MQTT 연결
    ↓
esp32/sensor 토픽 구독
    ↓
메시지 수신 (실시간)
    ↓
JSON 파싱
    ↓
UI 컴포넌트 업데이트
    ↓
사용자에게 표시
```

**Qt 예시 코드**:
```cpp
// MQTT 연결
QMQTT::Client client;
client.setHostName("broker.hivemq.com");
client.setPort(1883);
client.connectToHost();

// 토픽 구독
client.subscribe("esp32/sensor", 0);

// 메시지 수신 슬롯
void MainWindow::onMessageReceived(const QMQTT::Message &message) {
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(message.payload(), &error);
    
    if (error.error == QJsonParseError::NoError) {
        QJsonObject obj = doc.object();
        float temp = obj["temperature"].toDouble();
        float humi = obj["humidity"].toDouble();
        
        // UI 업데이트
        ui->temperatureLabel->setText(QString::number(temp) + "°C");
        ui->humidityLabel->setText(QString::number(humi) + "%");
        
        // 차트에 데이터 추가
        chart->addDataPoint(temp, humi);
    }
}
```

### 3. 데이터 로깅 (Node.js 서버)

**처리 흐름**:
```
MQTT 연결
    ↓
esp32/sensor 토픽 구독
    ↓
메시지 수신
    ↓
JSON 파싱 및 검증
    ↓
MongoDB 저장
    ↓
(선택) WebSocket으로 대시보드에 실시간 전송
```

**Node.js 예시 코드**:
```javascript
// MQTT 구독
mqttClient.subscribe('esp32/sensor', (err) => {
  if (!err) {
    console.log('✅ esp32/sensor 토픽 구독 완료');
  }
});

// 메시지 수신 처리
mqttClient.on('message', async (topic, message) => {
  if (topic === 'esp32/sensor') {
    try {
      const data = JSON.parse(message.toString());
      
      // MongoDB 저장 (온도)
      await db.saveSensorData({
        deviceId: data.deviceId || 'esp32-001',
        sensorType: 'temperature',
        value: data.temperature,
        unit: 'celsius',
        location: 'room1'
      });
      
      // MongoDB 저장 (습도)
      await db.saveSensorData({
        deviceId: data.deviceId || 'esp32-001',
        sensorType: 'humidity',
        value: data.humidity,
        unit: 'percent',
        location: 'room1'
      });
      
      // WebSocket으로 대시보드에 실시간 전송
      io.emit('sensor-data', data);
      
      console.log('📊 센서 데이터 저장 완료:', data);
    } catch (error) {
      console.error('❌ 센서 데이터 처리 실패:', error);
    }
  }
});
```

### 4. 로그 조회 (개발자)

**처리 흐름**:
```
HTTP GET 요청
    ↓
Node.js API 서버
    ↓
MongoDB 쿼리
    ↓
JSON 응답 반환
```

---

## 구현 상세

### Phase 1: ESP32에서 MQTT 발행

#### 1.1 app_main.c 수정

```c
// 센서 데이터 읽을 때마다 MQTT 발행
if (dht11_counter % 2 == 0) {
    dht11_data_t dht_data;
    esp_err_t ret = dht11_read(&dht_data);
    if (ret == ESP_OK && dht_data.valid) {
        // 평균 계산 모듈에 데이터 추가
        sensor_average_add(dht_data.temperature, dht_data.humidity);
        
        // MQTT로 센서 데이터 발행
        if (mqtt_manager_is_connected()) {
            char sensor_json[200];
            int64_t timestamp = esp_timer_get_time() / 1000000; // 초 단위
            
            snprintf(sensor_json, sizeof(sensor_json),
                "{"
                "\"deviceId\":\"esp32-001\","
                "\"temperature\":%.1f,"
                "\"humidity\":%.1f,"
                "\"timestamp\":%lld,"
                "\"valid\":true"
                "}",
                dht_data.temperature,
                dht_data.humidity,
                timestamp);
            
            mqtt_manager_publish(MQTT_TOPIC_SENSOR, sensor_json, 
                                strlen(sensor_json), 0, 0);
        }
    }
}
```

#### 1.2 필요한 헤더 추가
```c
#include "esp_timer.h"  // 타임스탬프 생성용
```

### Phase 2: Node.js 서버 확장

#### 2.1 server.js에 센서 토픽 구독 추가

```javascript
// MQTT 설정에 센서 토픽 추가
const MQTT_TOPIC_SENSOR = 'esp32/sensor';

mqttClient.on('connect', () => {
  console.log('✅ MQTT 브로커에 연결됨:', MQTT_BROKER);
  
  // 기존 토픽 구독
  mqttClient.subscribe(MQTT_TOPIC_STATUS);
  
  // 센서 토픽 구독 추가
  mqttClient.subscribe(MQTT_TOPIC_SENSOR, (err) => {
    if (!err) {
      console.log('📡 MQTT 토픽 구독:', MQTT_TOPIC_SENSOR);
    }
  });
});
```

#### 2.2 센서 데이터 처리 로직 추가

```javascript
mqttClient.on('message', async (topic, message) => {
  console.log(`📨 MQTT 메시지 수신 [${topic}]:`, message.toString());
  
  // 기존 처리 (esp32/status)
  if (topic === MQTT_TOPIC_STATUS) {
    io.emit('mqtt-message', {
      topic: topic,
      message: message.toString(),
      timestamp: new Date().toISOString()
    });
  }
  
  // 센서 데이터 처리 추가
  if (topic === MQTT_TOPIC_SENSOR) {
    try {
      const data = JSON.parse(message.toString());
      
      // MongoDB에 온도 데이터 저장
      await db.saveSensorData({
        deviceId: data.deviceId || 'esp32-001',
        sensorType: 'temperature',
        value: data.temperature,
        unit: 'celsius',
        location: 'room1'
      });
      
      // MongoDB에 습도 데이터 저장
      await db.saveSensorData({
        deviceId: data.deviceId || 'esp32-001',
        sensorType: 'humidity',
        value: data.humidity,
        unit: 'percent',
        location: 'room1'
      });
      
      // WebSocket으로 대시보드에 실시간 전송
      io.emit('sensor-data', {
        ...data,
        receivedAt: new Date().toISOString()
      });
      
      console.log('📊 센서 데이터 저장 완료:', data);
    } catch (error) {
      console.error('❌ 센서 데이터 처리 실패:', error.message);
    }
  }
});
```

### Phase 3: 라즈베리파이 QT 클라이언트

#### 3.1 프로젝트 설정

**CMakeLists.txt**:
```cmake
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Mqtt)

target_link_libraries(your_app
    Qt6::Core
    Qt6::Widgets
    Qt6::Mqtt
)
```

#### 3.2 MQTT 클라이언트 클래스

**SensorDataReceiver.h**:
```cpp
#ifndef SENSORDATARECEIVER_H
#define SENSORDATARECEIVER_H

#include <QObject>
#include <QMQTT/Client>

class SensorDataReceiver : public QObject
{
    Q_OBJECT

public:
    explicit SensorDataReceiver(QObject *parent = nullptr);
    ~SensorDataReceiver();

signals:
    void sensorDataReceived(float temperature, float humidity, qint64 timestamp);

private slots:
    void onConnected();
    void onDisconnected();
    void onMessageReceived(const QMQTT::Message &message);

private:
    QMQTT::Client *m_client;
};

#endif // SENSORDATARECEIVER_H
```

**SensorDataReceiver.cpp**:
```cpp
#include "SensorDataReceiver.h"
#include <QJsonDocument>
#include <QJsonObject>

SensorDataReceiver::SensorDataReceiver(QObject *parent)
    : QObject(parent)
{
    m_client = new QMQTT::Client(this);
    m_client->setHostName("broker.hivemq.com");
    m_client->setPort(1883);
    m_client->setClientId("raspberry-pi-qt-" + QString::number(qrand()));
    
    connect(m_client, &QMQTT::Client::connected, this, &SensorDataReceiver::onConnected);
    connect(m_client, &QMQTT::Client::disconnected, this, &SensorDataReceiver::onDisconnected);
    connect(m_client, &QMQTT::Client::received, this, &SensorDataReceiver::onMessageReceived);
    
    m_client->connectToHost();
}

void SensorDataReceiver::onConnected()
{
    qDebug() << "MQTT 연결됨";
    m_client->subscribe("esp32/sensor", 0);
}

void SensorDataReceiver::onDisconnected()
{
    qDebug() << "MQTT 연결 끊김";
}

void SensorDataReceiver::onMessageReceived(const QMQTT::Message &message)
{
    if (message.topic() == "esp32/sensor") {
        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(message.payload(), &error);
        
        if (error.error == QJsonParseError::NoError) {
            QJsonObject obj = doc.object();
            float temperature = obj["temperature"].toDouble();
            float humidity = obj["humidity"].toDouble();
            qint64 timestamp = obj["timestamp"].toVariant().toLongLong();
            
            emit sensorDataReceived(temperature, humidity, timestamp);
        }
    }
}

SensorDataReceiver::~SensorDataReceiver()
{
    if (m_client && m_client->isConnected()) {
        m_client->disconnectFromHost();
    }
}
```

#### 3.3 UI 업데이트 (MainWindow)

```cpp
// MainWindow.h
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onSensorDataReceived(float temperature, float humidity, qint64 timestamp);

private:
    Ui::MainWindow *ui;
    SensorDataReceiver *m_receiver;
    QChart *m_chart;
    QLineSeries *m_tempSeries;
    QLineSeries *m_humiSeries;
};

// MainWindow.cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    // 센서 데이터 수신자 초기화
    m_receiver = new SensorDataReceiver(this);
    connect(m_receiver, &SensorDataReceiver::sensorDataReceived,
            this, &MainWindow::onSensorDataReceived);
    
    // 차트 초기화
    m_chart = new QChart();
    m_tempSeries = new QLineSeries();
    m_tempSeries->setName("온도");
    m_humiSeries = new QLineSeries();
    m_humiSeries->setName("습도");
    
    m_chart->addSeries(m_tempSeries);
    m_chart->addSeries(m_humiSeries);
    m_chart->createDefaultAxes();
    
    ui->chartView->setChart(m_chart);
}

void MainWindow::onSensorDataReceived(float temperature, float humidity, qint64 timestamp)
{
    // UI 라벨 업데이트
    ui->temperatureLabel->setText(QString::number(temperature, 'f', 1) + "°C");
    ui->humidityLabel->setText(QString::number(humidity, 'f', 1) + "%");
    
    // 차트에 데이터 추가 (최근 100개만 유지)
    static int pointCount = 0;
    m_tempSeries->append(pointCount, temperature);
    m_humiSeries->append(pointCount, humidity);
    
    if (m_tempSeries->count() > 100) {
        m_tempSeries->remove(0);
        m_humiSeries->remove(0);
    }
    
    pointCount++;
    
    // 차트 축 업데이트
    m_chart->axes(Qt::Horizontal).first()->setRange(qMax(0, pointCount - 100), pointCount);
}
```

### Phase 4: HTTP API 활용 (개발자용)

#### 4.1 기존 API 활용

**센서 데이터 조회**:
```bash
# 최근 100개 데이터 조회
GET http://nodejs-server:3000/api/sensors/data?limit=100

# 특정 장치의 데이터만
GET http://nodejs-server:3000/api/sensors/data?deviceId=esp32-001&limit=50

# 온도 데이터만
GET http://nodejs-server:3000/api/sensors/data?sensorType=temperature&limit=100
```

**응답 예시**:
```json
{
  "success": true,
  "data": [
    {
      "_id": "...",
      "deviceId": "esp32-001",
      "sensorType": "temperature",
      "value": 25.5,
      "unit": "celsius",
      "location": "room1",
      "timestamp": "2024-01-01T12:00:00.000Z"
    },
    ...
  ],
  "count": 100,
  "timestamp": "2024-01-01T12:05:00.000Z"
}
```

#### 4.2 추가 API 엔드포인트 (선택)

**통계 API**:
```javascript
// server.js에 추가
app.get('/api/sensors/statistics', async (req, res) => {
  try {
    const deviceId = req.query.deviceId || 'esp32-001';
    const days = parseInt(req.query.days) || 7;
    const fromDate = new Date();
    fromDate.setDate(fromDate.getDate() - days);
    
    const tempData = await SensorData.find({
      deviceId: deviceId,
      sensorType: 'temperature',
      timestamp: { $gte: fromDate }
    }).sort({ timestamp: 1 });
    
    const humiData = await SensorData.find({
      deviceId: deviceId,
      sensorType: 'humidity',
      timestamp: { $gte: fromDate }
    }).sort({ timestamp: 1 });
    
    // 통계 계산
    const tempValues = tempData.map(d => d.value);
    const humiValues = humiData.map(d => d.value);
    
    res.json({
      success: true,
      statistics: {
        temperature: {
          min: Math.min(...tempValues),
          max: Math.max(...tempValues),
          avg: tempValues.reduce((a, b) => a + b, 0) / tempValues.length,
          count: tempValues.length
        },
        humidity: {
          min: Math.min(...humiValues),
          max: Math.max(...humiValues),
          avg: humiValues.reduce((a, b) => a + b, 0) / humiValues.length,
          count: humiValues.length
        },
        period: {
          from: fromDate.toISOString(),
          to: new Date().toISOString(),
          days: days
        }
      }
    });
  } catch (error) {
    res.status(500).json({ success: false, error: error.message });
  }
});
```

---

## API 명세

### ESP32 → MQTT 발행

**토픽**: `esp32/sensor`
**QoS**: 0
**메시지 형식**: JSON

```json
{
  "deviceId": "esp32-001",
  "temperature": 25.5,
  "humidity": 60.2,
  "timestamp": 1704067200,
  "valid": true
}
```

### Node.js 서버 API

#### GET /api/sensors/data
센서 데이터 조회

**Query Parameters**:
- `limit` (number, optional): 반환할 데이터 개수 (기본값: 100)
- `deviceId` (string, optional): 장치 ID 필터
- `sensorType` (string, optional): 센서 타입 필터 (temperature, humidity)
- `from` (ISO date, optional): 시작 날짜
- `to` (ISO date, optional): 종료 날짜

**응답**:
```json
{
  "success": true,
  "data": [...],
  "count": 100,
  "timestamp": "2024-01-01T12:00:00.000Z"
}
```

#### GET /api/sensors/statistics
센서 데이터 통계

**Query Parameters**:
- `deviceId` (string, optional): 장치 ID
- `days` (number, optional): 통계 기간 (일) (기본값: 7)

**응답**:
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
  }
}
```

---

## 설정 및 배포

### ESP32 설정

**config.h**:
```c
#define MQTT_BROKER_URL        "mqtt://broker.hivemq.com"
#define MQTT_BROKER_PORT       1883
#define MQTT_CLIENT_ID         "esp32-001"
#define MQTT_TOPIC_SENSOR      "esp32/sensor"
```

### Node.js 서버 설정

**.env**:
```env
MONGODB_URI=mongodb://localhost:27017/esp32_sensors
MQTT_BROKER=broker.hivemq.com
MQTT_PORT=1883
ESP32_IP=192.168.1.100
PORT=3000
```

### 라즈베리파이 QT 설정

**MQTT 연결 정보**:
- Broker: `broker.hivemq.com`
- Port: `1883`
- Topic: `esp32/sensor`

### 배포 순서

1. **ESP32 펌웨어 빌드 및 플래시**
   ```bash
   idf.py build flash monitor
   ```

2. **Node.js 서버 실행**
   ```bash
   cd esp32-server
   npm install
   npm start
   ```

3. **라즈베리파이 QT 애플리케이션 빌드 및 실행**
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ./your_app
   ```

---

## 트러블슈팅

### 문제 1: MQTT 연결 실패
- **원인**: 브로커 주소 또는 포트 오류
- **해결**: `broker.hivemq.com:1883` 확인

### 문제 2: 데이터가 저장되지 않음
- **원인**: MongoDB 연결 실패 또는 토픽 구독 실패
- **해결**: Node.js 서버 로그 확인, MongoDB 실행 상태 확인

### 문제 3: 라즈베리파이 QT에서 데이터 수신 안 됨
- **원인**: MQTT 클라이언트 연결 실패 또는 토픽 구독 오류
- **해결**: Qt 로그 확인, MQTT 클라이언트 연결 상태 확인

---

## 확장 가능성

### 향후 추가 가능한 기능

1. **다중 ESP32 지원**: deviceId로 구분하여 여러 장치 데이터 수집
2. **알람 기능**: 임계값 초과 시 MQTT로 알람 발행
3. **데이터 시각화**: Node.js 서버에 Grafana 연동
4. **모바일 앱**: MQTT 구독하여 모바일에서도 실시간 확인
5. **데이터 백업**: 주기적으로 CSV/JSON으로 데이터 내보내기

---

## 요약

이 아키텍처는 다음과 같은 장점을 제공합니다:

✅ **실시간성**: MQTT를 통한 즉각적인 데이터 전달
✅ **영구 저장**: MongoDB를 통한 데이터 로깅
✅ **확장성**: 여러 장치 및 클라이언트 지원 가능
✅ **분리된 책임**: 각 컴포넌트가 명확한 역할 수행
✅ **유연성**: 실시간 모니터링과 히스토리 분석 모두 지원


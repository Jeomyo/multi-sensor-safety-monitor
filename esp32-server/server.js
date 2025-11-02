require('dotenv').config();
const express = require('express');
const cors = require('cors');
const axios = require('axios');
const mqtt = require('mqtt');
const http = require('http');
const socketIo = require('socket.io');
const DatabaseService = require('./services/database');
const { SensorData } = require('./models');

// ZONES (in-memory; editable via PUT /api/zones)
let zones = [];

// Thresholds (in-memory, editable via PATCH /api/config)
const thresholds = {
  gas: parseFloat(process.env.GAS_THRESH || '200'),
  temp: parseFloat(process.env.TEMP_THRESH || '50'),
  humi: {
    min: parseFloat(process.env.HUMI_MIN || '20'),
    max: parseFloat(process.env.HUMI_MAX || '85')
  }
};

function parseNumber(v, fallback){
  const n = parseFloat(v);
  return Number.isFinite(n) ? n : fallback;
}

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"]
  }
});

// 데이터베이스 서비스 초기화
const db = new DatabaseService();

// ESP32 설정
const ESP32_IP = process.env.ESP32_IP || '172.30.1.92';
const ESP32_BASE_URL = `http://${ESP32_IP}`;

// MQTT 설정
const MQTT_BROKER = process.env.MQTT_BROKER || 'broker.hivemq.com';
const MQTT_PORT = parseInt(process.env.MQTT_PORT || '1883', 10);
const MQTT_TOPIC_LED = 'esp32/led';
const MQTT_TOPIC_STATUS = 'esp32/status';
const MQTT_TOPIC_SENSOR = 'esp32/sensor';

// 미들웨어 설정
app.use(cors());
app.use(express.json());
app.use(express.static('public'));

// MQTT 클라이언트 연결
const mqttClient = mqtt.connect(`mqtt://${MQTT_BROKER}:${MQTT_PORT}`, {
  clientId: 'nodejs-server-' + Math.random().toString(16).substr(2, 8)
});

mqttClient.on('connect', () => {
  console.log('✅ MQTT 브로커에 연결됨:', MQTT_BROKER);
  
  // ESP32 상태 토픽 구독
  mqttClient.subscribe(MQTT_TOPIC_STATUS, (err) => {
    if (!err) {
      console.log('📡 MQTT 토픽 구독:', MQTT_TOPIC_STATUS);
    }
  });
  
  // ESP32 센서 데이터 토픽 구독
  mqttClient.subscribe(MQTT_TOPIC_SENSOR, (err) => {
    if (!err) {
      console.log('📡 MQTT 토픽 구독:', MQTT_TOPIC_SENSOR);
    }
  });
});

mqttClient.on('message', async (topic, message) => {
  console.log(`📨 MQTT 메시지 수신 [${topic}]:`, message.toString());
  
  // 상태 토픽 처리 (기존)
  if (topic === MQTT_TOPIC_STATUS) {
    // 웹소켓으로 클라이언트들에게 실시간 데이터 전송
    io.emit('mqtt-message', {
      topic: topic,
      message: message.toString(),
      timestamp: new Date().toISOString()
    });
  }
  
  // 센서 데이터 토픽 처리
  if (topic === MQTT_TOPIC_SENSOR) {
    try {
      const data = JSON.parse(message.toString());
      
      // 데이터 검증
      if (!data.temperature && !data.humidity) {
        console.warn('⚠️ 센서 데이터에 temperature 또는 humidity가 없음');
        return;
      }
      
      const deviceId = data.deviceId || 'esp32-001';
      const location = data.location || 'room1';
      
      // MongoDB에 온도 데이터 저장
      if (typeof data.temperature === 'number' && !isNaN(data.temperature)) {
        await db.saveSensorData({
          deviceId: deviceId,
          sensorType: 'temperature',
          value: data.temperature,
          unit: 'celsius',
          location: location
        });
      }
      
      // MongoDB에 습도 데이터 저장
      if (typeof data.humidity === 'number' && !isNaN(data.humidity)) {
        await db.saveSensorData({
          deviceId: deviceId,
          sensorType: 'humidity',
          value: data.humidity,
          unit: 'percent',
          location: location
        });
      }
      
      // WebSocket으로 대시보드에 실시간 전송
      io.emit('sensor-data', {
        ...data,
        receivedAt: new Date().toISOString()
      });
      
      // 기존 mqtt-message 이벤트도 함께 전송 (호환성)
      io.emit('mqtt-message', {
        topic: topic,
        message: message.toString(),
        timestamp: new Date().toISOString()
      });
      
      console.log('📊 센서 데이터 저장 완료:', {
        deviceId: deviceId,
        temperature: data.temperature,
        humidity: data.humidity
      });
    } catch (error) {
      console.error('❌ 센서 데이터 처리 실패:', error.message);
      console.error('원본 메시지:', message.toString());
    }
  }
});

// ESP32 상태 조회 API
app.get('/api/esp32/status', async (req, res) => {
  try {
    const response = await axios.get(`${ESP32_BASE_URL}/status`);
    res.json({
      success: true,
      data: response.data,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('ESP32 상태 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: 'ESP32 연결 실패',
      message: error.message
    });
  }
});

// ESP32 LED 제어 API
app.post('/api/esp32/led', async (req, res) => {
  const startTime = Date.now();
  const clientIP = req.ip || req.connection.remoteAddress;
  const sessionId = req.headers['x-session-id'] || 'unknown';
  
  try {
    const { state } = req.body;
    
    // ESP32에 직접 요청
    const response = await axios.post(`${ESP32_BASE_URL}/led`, {
      state: state
    });
    
    // MQTT로도 메시지 발행
    const mqttMessage = state ? 'on' : 'off';
    mqttClient.publish(MQTT_TOPIC_LED, mqttMessage);
    
    // 데이터베이스에 LED 제어 이력 저장
    if (db.isConnected) {
      await db.saveLedControl({
        action: state ? 'on' : 'off',
        state: state,
        source: 'api',
        ip: clientIP,
        metadata: {
          userAgent: req.headers['user-agent'],
          sessionId: sessionId,
          requestId: req.headers['x-request-id'] || 'unknown'
        }
      });
      
      // 사용자 활동 저장
      await db.saveUserActivity({
        sessionId: sessionId,
        action: `led_${state ? 'on' : 'off'}`,
        resource: 'esp32/led',
        ip: clientIP,
        userAgent: req.headers['user-agent'],
        duration: Date.now() - startTime,
        success: true
      });
    }
    
    res.json({
      success: true,
      data: response.data,
      mqttPublished: true,
      databaseSaved: db.isConnected,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('ESP32 LED 제어 실패:', error.message);
    
    // 에러 이벤트 저장
    if (db.isConnected) {
      await db.saveSystemEvent({
        eventType: 'error',
        message: `LED 제어 실패: ${error.message}`,
        level: 'error',
        metadata: {
          ip: clientIP,
          errorCode: error.code || 'UNKNOWN'
        }
      });
    }
    
    res.status(500).json({
      success: false,
      error: 'LED 제어 실패',
      message: error.message
    });
  }
});

// 알람 저장 API (경량)
app.post('/api/alarms', async (req, res) => {
  try {
    const payload = req.body || {};
    if (db.isConnected) {
      await db.saveSystemEvent({
        deviceId: payload.deviceId || 'esp32-001',
        eventType: 'warning',
        message: payload.message || 'Alarm',
        level: payload.level || 'warn',
        metadata: payload.metadata || {}
      });
    }
    res.json({ success: true, saved: db.isConnected });
  } catch (e) {
    res.status(500).json({ success: false, message: e.message });
  }
});

// MQTT 메시지 발행 API
app.post('/api/mqtt/publish', (req, res) => {
  try {
    const { topic, message } = req.body;
    
    mqttClient.publish(topic, message);
    
    res.json({
      success: true,
      message: 'MQTT 메시지 발행 완료',
      topic: topic,
      payload: message,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('MQTT 메시지 발행 실패:', error.message);
    res.status(500).json({
      success: false,
      error: 'MQTT 발행 실패',
      message: error.message
    });
  }
});

// ZONES API
app.get('/api/zones', (req, res) => {
  res.json({ success: true, zones, timestamp: new Date().toISOString() });
});

app.put('/api/zones', (req, res) => {
  try {
    const body = req.body || {};
    if (!Array.isArray(body.zones)) {
      return res.status(400).json({ success: false, message: 'zones must be array' });
    }
    zones = body.zones.map((z, i) => ({
      id: z.id || `zone-${i+1}`,
      name: z.name || `Zone ${i+1}`,
      x: Number(z.x) || 0,
      y: Number(z.y) || 0,
      w: Number(z.w) || 120,
      h: Number(z.h) || 90
    }));
    res.json({ success: true, count: zones.length });
  } catch (e) {
    res.status(400).json({ success: false, message: e.message });
  }
});

// 설정 조회 API
app.get('/api/config', (req, res) => {
  res.json({
    thresholds,
    topics: {
      status: MQTT_TOPIC_STATUS,
      led: MQTT_TOPIC_LED,
      sensor: MQTT_TOPIC_SENSOR,
      sensors: process.env.SENSOR_TOPIC || 'esp32/sensors'
    },
    timestamp: new Date().toISOString()
  });
});

// 임계치 업데이트 API
app.patch('/api/config', (req, res) => {
  try {
    const b = req.body || {};
    if (b.thresholds) {
      if (typeof b.thresholds.gas !== 'undefined') thresholds.gas = parseNumber(b.thresholds.gas, thresholds.gas);
      if (typeof b.thresholds.temp !== 'undefined') thresholds.temp = parseNumber(b.thresholds.temp, thresholds.temp);
      if (b.thresholds.humi) {
        if (typeof b.thresholds.humi.min !== 'undefined') thresholds.humi.min = parseNumber(b.thresholds.humi.min, thresholds.humi.min);
        if (typeof b.thresholds.humi.max !== 'undefined') thresholds.humi.max = parseNumber(b.thresholds.humi.max, thresholds.humi.max);
      }
    }
    res.json({ success: true, thresholds });
  } catch (e) {
    res.status(400).json({ success: false, message: e.message });
  }
});

// 시스템 정보 API
app.get('/api/system/info', (req, res) => {
  res.json({
    server: 'Node.js ESP32 Control Server',
    version: '1.0.0',
    esp32: {
      ip: ESP32_IP,
      baseUrl: ESP32_BASE_URL,
      status: 'connected'
    },
    mqtt: {
      broker: MQTT_BROKER,
      port: MQTT_PORT,
      connected: mqttClient.connected
    },
    database: db.getStatus(),
    timestamp: new Date().toISOString()
  });
});

// LED 제어 이력 조회 API
app.get('/api/history/led', async (req, res) => {
  try {
    const limit = parseInt(req.query.limit) || 50;
    const deviceId = req.query.deviceId || 'esp32-001';
    
    const history = await db.getLedControlHistory(limit, deviceId);
    
    res.json({
      success: true,
      data: history,
      count: history.length,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('LED 이력 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: 'LED 이력 조회 실패',
      message: error.message
    });
  }
});

// 센서 데이터 조회 API
app.get('/api/sensors/data', async (req, res) => {
  try {
    const limit = parseInt(req.query.limit) || 100;
    const deviceId = req.query.deviceId || 'esp32-001';
    const sensorType = req.query.sensorType || null;
    const from = req.query.from || null;
    const to = req.query.to || null;
    
    let query = { deviceId };
    if (sensorType) query.sensorType = sensorType;
    if (from || to) {
      query.timestamp = {};
      if (from) query.timestamp.$gte = new Date(from);
      if (to) query.timestamp.$lte = new Date(to);
    }
    
    const data = await SensorData
      .find(query)
      .sort({ timestamp: -1 })
      .limit(limit)
      .lean();
    
    res.json({
      success: true,
      data: data,
      count: data.length,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('센서 데이터 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: '센서 데이터 조회 실패',
      message: error.message
    });
  }
});

// 센서 데이터 통계 API
app.get('/api/sensors/statistics', async (req, res) => {
  try {
    const deviceId = req.query.deviceId || 'esp32-001';
    const days = parseInt(req.query.days) || 7;
    
    const statistics = await db.getSensorStatistics(deviceId, days);
    
    res.json({
      success: true,
      statistics: statistics,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('센서 통계 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: '센서 통계 조회 실패',
      message: error.message
    });
  }
});

// 시스템 이벤트 조회 API
app.get('/api/system/events', async (req, res) => {
  try {
    const limit = parseInt(req.query.limit) || 100;
    const deviceId = req.query.deviceId || 'esp32-001';
    const level = req.query.level || null;
    const from = req.query.from || null;
    const to = req.query.to || null;
    
    const events = await db.getSystemEvents(limit, deviceId, level, from, to);
    
    res.json({
      success: true,
      data: events,
      count: events.length,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('시스템 이벤트 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: '시스템 이벤트 조회 실패',
      message: error.message
    });
  }
});

// 통계 데이터 조회 API
app.get('/api/statistics', async (req, res) => {
  try {
    const deviceId = req.query.deviceId || 'esp32-001';
    const days = parseInt(req.query.days) || 7;
    
    const statistics = await db.getStatistics(deviceId, days);
    
    res.json({
      success: true,
      data: statistics,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('통계 데이터 조회 실패:', error.message);
    res.status(500).json({
      success: false,
      error: '통계 데이터 조회 실패',
      message: error.message
    });
  }
});

// 종료 시 정리
function setupGracefulShutdown() {
  const shutdown = async () => {
    try {
      console.log('\n🛑 종료 시그널 수신: 정리 중...');
      if (mqttClient && mqttClient.end) {
        try { mqttClient.end(true); } catch (e) {}
      }
      if (db && db.disconnect) {
        try { await db.disconnect(); } catch (e) {}
      }
      process.exit(0);
    } catch (e) {
      process.exit(1);
    }
  };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}
setupGracefulShutdown();

// 웹소켓 연결 처리
io.on('connection', (socket) => {
  console.log('🌐 클라이언트 연결됨:', socket.id);
  
  // 클라이언트에게 현재 상태 전송
  socket.emit('server-status', {
    message: '서버에 연결되었습니다',
    timestamp: new Date().toISOString()
  });
  
  socket.on('disconnect', () => {
    console.log('❌ 클라이언트 연결 해제:', socket.id);
  });
});

// 서버 시작
const DEFAULT_PORT = parseInt(process.env.PORT || '3000', 10);
function normalizePort(p){ const n = Number(p); return Number.isNaN(n)? DEFAULT_PORT : n; }

async function startServer() {
  try {
    // 데이터베이스 연결
    await db.connect();
    
    // 서버 시작 (포트 충돌 시 자동 증가)
    let port = DEFAULT_PORT;
    const tryListen = () => {
      const srv = server.listen(port, () => {
        console.log('🚀 ESP32 제어 서버 시작됨!');
        console.log(`📡 서버 주소: http://localhost:${port} `);
        console.log(`🔗 ESP32 주소: ${ESP32_BASE_URL}`);
        console.log(`📨 MQTT 브로커: ${MQTT_BROKER}:${MQTT_PORT}`);
        console.log(`🗄️ 데이터베이스: ${db.isConnected ? '연결됨' : '연결 실패'}`);
        console.log('='.repeat(50));
      });
      srv.on('error', (err) => {
        if (err.code === 'EADDRINUSE') {
          console.warn(`⚠️ 포트 ${port} 사용 중, 다음 포트 시도`);
          port += 1;
          tryListen();
        } else {
          console.error('❌ 서버 시작 실패:', err.message);
          process.exit(1);
        }
      });
    };
    tryListen();
  } catch (error) {
    console.error('❌ 서버 시작 실패:', error.message);
    process.exit(1);
  }
}

// 서버 시작
startServer();

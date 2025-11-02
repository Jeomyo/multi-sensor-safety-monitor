const mongoose = require('mongoose');

// LED 제어 이력 스키마
const ledControlSchema = new mongoose.Schema({
  deviceId: {
    type: String,
    required: true,
    default: 'esp32-001'
  },
  action: {
    type: String,
    required: true,
    enum: ['on', 'off', 'toggle']
  },
  state: {
    type: Boolean,
    required: true
  },
  source: {
    type: String,
    required: true,
    enum: ['web', 'mqtt', 'api', 'mobile']
  },
  ip: {
    type: String,
    required: true
  },
  timestamp: {
    type: Date,
    default: Date.now
  },
  metadata: {
    userAgent: String,
    sessionId: String,
    requestId: String
  }
});

// 센서 데이터 스키마 (향후 확장용)
const sensorDataSchema = new mongoose.Schema({
  deviceId: {
    type: String,
    required: true,
    default: 'esp32-001'
  },
  sensorType: {
    type: String,
    required: true,
    enum: ['temperature', 'humidity', 'light', 'motion', 'distance']
  },
  value: {
    type: Number,
    required: true
  },
  unit: {
    type: String,
    required: true
  },
  timestamp: {
    type: Date,
    default: Date.now
  },
  location: {
    type: String,
    default: 'unknown'
  }
});

// 시스템 이벤트 스키마
const systemEventSchema = new mongoose.Schema({
  deviceId: {
    type: String,
    required: true,
    default: 'esp32-001'
  },
  eventType: {
    type: String,
    required: true,
    enum: ['connection', 'disconnection', 'error', 'warning', 'info']
  },
  message: {
    type: String,
    required: true
  },
  level: {
    type: String,
    required: true,
    enum: ['debug', 'info', 'warn', 'error']
  },
  timestamp: {
    type: Date,
    default: Date.now
  },
  metadata: {
    ip: String,
    mqttTopic: String,
    errorCode: String
  }
});

// 사용자 활동 스키마
const userActivitySchema = new mongoose.Schema({
  userId: {
    type: String,
    default: 'anonymous'
  },
  sessionId: {
    type: String,
    required: true
  },
  action: {
    type: String,
    required: true
  },
  resource: {
    type: String,
    required: true
  },
  ip: {
    type: String,
    required: true
  },
  userAgent: String,
  timestamp: {
    type: Date,
    default: Date.now
  },
  duration: Number, // 액션 수행 시간 (ms)
  success: {
    type: Boolean,
    default: true
  }
});

// 인덱스 생성
ledControlSchema.index({ timestamp: -1 });
ledControlSchema.index({ deviceId: 1, timestamp: -1 });

sensorDataSchema.index({ timestamp: -1 });
sensorDataSchema.index({ deviceId: 1, sensorType: 1, timestamp: -1 });

systemEventSchema.index({ timestamp: -1 });
systemEventSchema.index({ deviceId: 1, eventType: 1, timestamp: -1 });

userActivitySchema.index({ timestamp: -1 });
userActivitySchema.index({ userId: 1, timestamp: -1 });

// 모델 생성
const LedControl = mongoose.model('LedControl', ledControlSchema);
const SensorData = mongoose.model('SensorData', sensorDataSchema);
const SystemEvent = mongoose.model('SystemEvent', systemEventSchema);
const UserActivity = mongoose.model('UserActivity', userActivitySchema);

module.exports = {
  LedControl,
  SensorData,
  SystemEvent,
  UserActivity
};



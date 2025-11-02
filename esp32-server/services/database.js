const mongoose = require('mongoose');
const { LedControl, SensorData, SystemEvent, UserActivity } = require('../models');

class DatabaseService {
  constructor() {
    this.isConnected = false;
    // MongoDB Atlas 또는 로컬 MongoDB 사용
    this.connectionString = process.env.MONGODB_URI || 'mongodb://localhost:27017/esp32-iot';
  }

  async connect() {
    try {
      await mongoose.connect(this.connectionString, {});
      mongoose.set('strictQuery', true);
      
      this.isConnected = true;
      console.log('✅ MongoDB 연결 성공:', this.connectionString);
      
      // 연결 이벤트 리스너
      mongoose.connection.on('error', (err) => {
        console.error('❌ MongoDB 연결 오류:', err);
        this.isConnected = false;
      });
      
      mongoose.connection.on('disconnected', () => {
        console.log('⚠️ MongoDB 연결 끊어짐');
        this.isConnected = false;
      });
      
      mongoose.connection.on('reconnected', () => {
        console.log('🔄 MongoDB 재연결됨');
        this.isConnected = true;
      });
      
    } catch (error) {
      console.error('❌ MongoDB 연결 실패:', error.message);
      this.isConnected = false;
    }
  }

  async disconnect() {
    try {
      await mongoose.disconnect();
      this.isConnected = false;
      console.log('📴 MongoDB 연결 해제됨');
    } catch (error) {
      console.error('❌ MongoDB 연결 해제 실패:', error.message);
    }
  }

  // LED 제어 이력 저장
  async saveLedControl(data) {
    try {
      const ledControl = new LedControl({
        deviceId: data.deviceId || 'esp32-001',
        action: data.action,
        state: data.state,
        source: data.source,
        ip: data.ip,
        metadata: data.metadata || {}
      });
      
      const saved = await ledControl.save();
      console.log('💾 LED 제어 이력 저장됨:', saved._id);
      return saved;
    } catch (error) {
      console.error('❌ LED 제어 이력 저장 실패:', error.message);
      throw error;
    }
  }

  // 센서 데이터 저장
  async saveSensorData(data) {
    try {
      const sensorData = new SensorData({
        deviceId: data.deviceId || 'esp32-001',
        sensorType: data.sensorType,
        value: data.value,
        unit: data.unit,
        location: data.location || 'unknown'
      });
      
      const saved = await sensorData.save();
      console.log('📊 센서 데이터 저장됨:', saved._id);
      return saved;
    } catch (error) {
      console.error('❌ 센서 데이터 저장 실패:', error.message);
      throw error;
    }
  }

  // 시스템 이벤트 저장
  async saveSystemEvent(data) {
    try {
      const systemEvent = new SystemEvent({
        deviceId: data.deviceId || 'esp32-001',
        eventType: data.eventType,
        message: data.message,
        level: data.level,
        metadata: data.metadata || {}
      });
      
      const saved = await systemEvent.save();
      console.log('📝 시스템 이벤트 저장됨:', saved._id);
      return saved;
    } catch (error) {
      console.error('❌ 시스템 이벤트 저장 실패:', error.message);
      throw error;
    }
  }

  // 사용자 활동 저장
  async saveUserActivity(data) {
    try {
      const userActivity = new UserActivity({
        userId: data.userId || 'anonymous',
        sessionId: data.sessionId,
        action: data.action,
        resource: data.resource,
        ip: data.ip,
        userAgent: data.userAgent,
        duration: data.duration,
        success: data.success !== false
      });
      
      const saved = await userActivity.save();
      console.log('👤 사용자 활동 저장됨:', saved._id);
      return saved;
    } catch (error) {
      console.error('❌ 사용자 활동 저장 실패:', error.message);
      throw error;
    }
  }

  // LED 제어 이력 조회
  async getLedControlHistory(limit = 50, deviceId = 'esp32-001') {
    try {
      const history = await LedControl
        .find({ deviceId })
        .sort({ timestamp: -1 })
        .limit(limit)
        .lean();
      
      return history;
    } catch (error) {
      console.error('❌ LED 제어 이력 조회 실패:', error.message);
      throw error;
    }
  }

  // 센서 데이터 조회
  async getSensorData(limit = 100, deviceId = 'esp32-001', sensorType = null) {
    try {
      const query = { deviceId };
      if (sensorType) query.sensorType = sensorType;
      
      const data = await SensorData
        .find(query)
        .sort({ timestamp: -1 })
        .limit(limit)
        .lean();
      
      return data;
    } catch (error) {
      console.error('❌ 센서 데이터 조회 실패:', error.message);
      throw error;
    }
  }

  // 센서 데이터 통계 조회
  async getSensorStatistics(deviceId = 'esp32-001', days = 7) {
    try {
      const startDate = new Date();
      startDate.setDate(startDate.getDate() - days);
      
      // 온도 데이터 통계
      const tempData = await SensorData.find({
        deviceId: deviceId,
        sensorType: 'temperature',
        timestamp: { $gte: startDate }
      }).sort({ timestamp: 1 }).lean();
      
      // 습도 데이터 통계
      const humiData = await SensorData.find({
        deviceId: deviceId,
        sensorType: 'humidity',
        timestamp: { $gte: startDate }
      }).sort({ timestamp: 1 }).lean();
      
      // 통계 계산
      const tempValues = tempData.map(d => d.value);
      const humiValues = humiData.map(d => d.value);
      
      return {
        temperature: {
          min: tempValues.length > 0 ? Math.min(...tempValues) : null,
          max: tempValues.length > 0 ? Math.max(...tempValues) : null,
          avg: tempValues.length > 0 ? tempValues.reduce((a, b) => a + b, 0) / tempValues.length : null,
          count: tempValues.length
        },
        humidity: {
          min: humiValues.length > 0 ? Math.min(...humiValues) : null,
          max: humiValues.length > 0 ? Math.max(...humiValues) : null,
          avg: humiValues.length > 0 ? humiValues.reduce((a, b) => a + b, 0) / humiValues.length : null,
          count: humiValues.length
        },
        period: {
          from: startDate.toISOString(),
          to: new Date().toISOString(),
          days: days
        }
      };
    } catch (error) {
      console.error('❌ 센서 통계 조회 실패:', error.message);
      throw error;
    }
  }

  // 시스템 이벤트 조회
  async getSystemEvents(limit = 100, deviceId = 'esp32-001', level = null, from = null, to = null) {
    try {
      const query = { deviceId };
      if (level) query.level = level;
      if (from || to) {
        query.timestamp = {};
        if (from) query.timestamp.$gte = new Date(from);
        if (to) query.timestamp.$lte = new Date(to);
      }
      const events = await SystemEvent
        .find(query)
        .sort({ timestamp: -1 })
        .limit(limit)
        .lean();
      return events;
    } catch (error) {
      console.error('❌ 시스템 이벤트 조회 실패:', error.message);
      throw error;
    }
  }

  // 통계 데이터 조회
  async getStatistics(deviceId = 'esp32-001', days = 7) {
    try {
      const startDate = new Date();
      startDate.setDate(startDate.getDate() - days);
      
      const [
        ledControls,
        sensorData,
        systemEvents,
        userActivities
      ] = await Promise.all([
        LedControl.countDocuments({ 
          deviceId, 
          timestamp: { $gte: startDate } 
        }),
        SensorData.countDocuments({ 
          deviceId, 
          timestamp: { $gte: startDate } 
        }),
        SystemEvent.countDocuments({ 
          deviceId, 
          timestamp: { $gte: startDate } 
        }),
        UserActivity.countDocuments({ 
          timestamp: { $gte: startDate } 
        })
      ]);
      
      return {
        period: `${days}일`,
        ledControls,
        sensorData,
        systemEvents,
        userActivities,
        totalRecords: ledControls + sensorData + systemEvents + userActivities
      };
    } catch (error) {
      console.error('❌ 통계 데이터 조회 실패:', error.message);
      throw error;
    }
  }

  // 데이터베이스 상태 확인
  getStatus() {
    return {
      connected: this.isConnected,
      connectionString: this.connectionString,
      readyState: mongoose.connection.readyState
    };
  }
}

module.exports = DatabaseService;


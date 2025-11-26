#include "sensor_data_provider.h"
#include <QtMqtt/QMqttTopicFilter>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

// ✅ 추가
#include <QDateTime>
#include "SystemMonitor.h"
extern SystemMonitor* g_systemMonitor;   // ✅ 전역 객체 참조

// ✅ 기존 코드 그대로 유지
SensorDataProvider::SensorDataProvider(QObject *parent)
    : QObject(parent),
    m_client(new QMqttClient(this))
{
    connect(m_client, &QMqttClient::messageReceived,
            this, &SensorDataProvider::handleMessage);
}

void SensorDataProvider::connectToBroker(const QString &host, int port)
{
    m_client->setHostname(host);
    m_client->setPort(port);
    m_client->setKeepAlive(60);
    m_client->connectToHost();

    connect(m_client, &QMqttClient::connected, this, [this]() {
        qDebug() << "✅ MQTT 연결 성공!";
        // 기존 환경 데이터 구독
        m_client->subscribe(QMqttTopicFilter("factory/temperature"));
        m_client->subscribe(QMqttTopicFilter("factory/humidity"));
        m_client->subscribe(QMqttTopicFilter("factory/dust"));

        // ✅ 추가: 지도 관련 토픽 구독
        m_client->subscribe(QMqttTopicFilter("/map/data"));
        m_client->subscribe(QMqttTopicFilter("/obstacle/data"));
        m_client->subscribe(QMqttTopicFilter("/worker/data"));

        // ✅ 추가: 선민 Gesture 관련 토픽 구독
        m_client->subscribe(QMqttTopicFilter("/system/mode/gotowork"), 1);
        m_client->subscribe(QMqttTopicFilter("/system/mode/leavework"), 1);
        m_client->subscribe(QMqttTopicFilter("/system/msg"));
    });

    connect(m_client, &QMqttClient::disconnected, this, [this]() {
        qDebug() << "❌ MQTT 연결 끊김!";
        QTimer::singleShot(3000, this, [this]() {
            m_client->connectToHost();
        });
    });
}

// ✅ 기존 handleMessage에 지도 관련 메시지 처리 추가
void SensorDataProvider::handleMessage(const QByteArray &message, const QMqttTopicName &topic)
{
    qDebug() << "🔥 RAW TOPIC =" << topic.name();
    qDebug() << "🔥 RAW MSG   =" << message;
    QString topicName = topic.name();

    // ✅ 추가: MQTT 지연 계산
    static qint64 lastRecv = 0;
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (lastRecv != 0 && g_systemMonitor) {
        double latency = static_cast<double>(now - lastRecv);
        g_systemMonitor->updateMqttLatency(latency);
        qDebug() << "📡 MQTT 지연:" << latency << "ms";
    }
    lastRecv = now;

    // -------------------------------
    // ① 기존 환경 데이터 처리
    // -------------------------------
    if (topicName == "factory/temperature") {
        m_temperature = message.toDouble();
        emit temperatureChanged();
    }
    else if (topicName == "factory/humidity") {
        m_humidity = message.toDouble();
        emit humidityChanged();
    }
    else if (topicName == "factory/dust") {
        m_dust = message.toDouble();
        emit dustChanged();
    }

    // -------------------------------
    // ② 추가: 지도 관련 토픽 처리
    // -------------------------------
    /*
    else if (topicName == "/map/data") {
        QJsonDocument doc = QJsonDocument::fromJson(message);
        if (doc.isArray()) {
            QJsonArray arr = doc.array();
            emit mapUpdated(arr);
            qDebug() << "🗺️ 맵 데이터 수신:" << arr;
        }
    }
    */
    else if (topicName == "/map/data") {
        QString utf8Msg = QString::fromUtf8(message);
        QJsonDocument doc = QJsonDocument::fromJson(utf8Msg.toUtf8());
        if (!doc.isArray()) {
            qDebug() << "❌ JSON 파싱 실패!" << utf8Msg;
            return;
        }
        QJsonArray arr = doc.array();
        emit mapUpdated(arr);
        qDebug() << "✅ 맵 데이터 파싱 성공! 갯수 =" << arr.size();
    }
    else if (topicName == "/obstacle/data") {
        QJsonDocument doc = QJsonDocument::fromJson(message);
        if (doc.isArray()) {
            QJsonArray arr = doc.array();
            emit obstaclesUpdated(arr);
            qDebug() << "🧱 장애물 데이터 수신:" << arr;
        }
    }
    else if (topicName == "/worker/data") {
        QJsonDocument doc = QJsonDocument::fromJson(message);
        if (doc.isArray()) {
            QJsonArray arr = doc.array();
            emit workersUpdated(arr);
            qDebug() << "🧍 작업자 데이터 수신:" << arr;
        }
    }
    else if (topicName == "/system/mode/gotowork") {
        int value = QString(message).toInt();
        emit workStartModeChanged(value);
    }

    else if (topicName == "/system/mode/leavework") {
        int value = QString(message).toInt();
        emit workEndModeChanged(value);
    }
    else if (topicName == "/system/msg") {
        QString msg = QString(message);
        emit systemMsgOneReceived(msg);
    }



    qDebug() << "📩 수신됨:" << topicName << "=" << message;
}

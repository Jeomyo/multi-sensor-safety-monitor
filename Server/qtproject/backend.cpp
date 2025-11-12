#include "backend.h"
#include <QDebug>
#include <QtMqtt/QMqttTopicFilter>

Backend::Backend(QObject *parent)
    : QObject(parent)
{
}

void Backend::login(const QString &id, const QString &pw)
{
    qDebug() << "로그인 시도:" << id << pw;

    if (id == "jeomyo" && pw == "jeomyo") {
        qDebug() << "로그인 성공";
        emit loginSuccess();     // QML로 시그널 전송
    } else {
        qDebug() << "로그인 실패";
        emit loginFailed();
    }
}

void Backend::updateData(double newValue)
{
    qDebug() << "새로운 데이터 수신:" << newValue;

    // QML에 실시간으로 값 전달
    emit gauge1ValueChanged(newValue);
}

void Backend::setupMqtt()
{
    client = new QMqttClient(this);
    client->setHostname("test.mosquitto.org");
    client->setPort(1883);

    connect(client, &QMqttClient::connected, this, [this]() {
        qDebug() << "✅ MQTT 연결 성공";
        client->subscribe(QMqttTopicFilter(QStringLiteral("ajou/mqtttest/value")), 0);
    });

    connect(client, &QMqttClient::messageReceived, this,
            [this](const QByteArray &message, const QMqttTopicName &topic) {
                QString val = QString::fromUtf8(message);
                emit newMqttValue(val);
                emit gauge1ValueChanged(val.toDouble());
                qDebug() << "📩 수신됨:" << topic.name() << val;
            });

    client->connectToHost();
}

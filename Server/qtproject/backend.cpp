#include "backend.h"
#include <QDebug>
#include <QtMqtt/QMqttTopicFilter>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

Backend::Backend(QObject *parent)
    : QObject(parent)
{
    loadAccountsFromFile();   // ✅ 프로그램 시작 시 로그인 정보 로드
}

void Backend::login(const QString &id, const QString &pw)
{
    qDebug() << "로그인 시도:" << id << pw;

    if (accounts.contains(id) && accounts.value(id) == pw) {
        qDebug() << "로그인 성공";
        emit loginSuccess();
    } else {
        qDebug() << "로그인 실패";
        emit loginFailed();
    }
}

void Backend::loadAccountsFromFile()
{
    QFile file("accounts.json");
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "❌ accounts.json 열기 실패:" << file.errorString();
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray()) {
        qWarning() << "❌ JSON 구조 오류 (배열이 아님)";
        return;
    }

    QJsonArray arr = doc.array();
    accounts.clear();

    for (const QJsonValue &v : arr) {
        if (!v.isObject()) continue;
        QJsonObject o = v.toObject();
        QString id = o["id"].toString();
        QString pw = o["pw"].toString();
        if (!id.isEmpty())
            accounts.insert(id, pw);
    }

    qDebug() << "✅ 계정 로드 완료, 총" << accounts.size() << "개";
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

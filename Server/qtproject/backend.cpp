#include "backend.h"

#include <QDebug>
#include <QTimer>
#include <QRandomGenerator>
#include <QtMqtt/QMqttClient>
#include <QtMqtt/QMqttTopicFilter>

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QDir>
#include <QCoreApplication>
#include <QProcess>

// ----------------------
// 생성자
// ----------------------
Backend::Backend(QObject *parent)
    : QObject(parent)
{
    // MQTT 객체 생성
    client = new QMqttClient(this);

    // 계정 파일 로드
    loadAccountsFromFile();

    // 현재 시간 자동 갱신 (1초 주기)
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        m_currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
        emit currentTimeChanged();
    });
    timer->start(1000);
}

// ----------------------
// 로그인 기능
// ----------------------
void Backend::login(const QString &id, const QString &pw)
{
    bool success = (accounts.contains(id) && accounts.value(id) == pw);

    // 로그인 기록 저장
    writeLoginLog(id, pw, success);

    if (success)
        emit loginSuccess();
    else
        emit loginFailed();
}

// ----------------------
// JSON 계정 파일 로드
// ----------------------
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
        qWarning() << "❌ JSON 구조 오류 (배열 아님)";
        return;
    }

    accounts.clear();
    QJsonArray arr = doc.array();

    for (auto v : arr) {
        if (!v.isObject()) continue;
        QJsonObject o = v.toObject();
        QString id = o["id"].toString();
        QString pw = o["pw"].toString();
        if (!id.isEmpty())
            accounts[id] = pw;
    }
}

// ----------------------
// 로그인 로그 기록
// ----------------------
void Backend::writeLoginLog(const QString &id, const QString &pw, bool success)
{
    QString logPath = QCoreApplication::applicationDirPath() + "/../../log/logins.json";
    QFile file(logPath);

    QJsonArray logArray;

    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        if (doc.isArray())
            logArray = doc.array();
        file.close();
    }

    QJsonObject log;
    log["timestamp"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    log["id"] = id;
    log["pw"] = pw;
    log["success"] = success;

    logArray.append(log);

    if (file.open(QIODevice::WriteOnly)) {
        QJsonDocument saveDoc(logArray);
        file.write(saveDoc.toJson(QJsonDocument::Indented));
        file.close();
    }
}

// ----------------------
// MQTT 초기 설정
// ----------------------
void Backend::setupMqtt()
{
    client->setHostname("broker.hivemq.com");
    client->setPort(1883);
    client->setProtocolVersion(QMqttClient::MQTT_3_1_1);

    QString clientId = "QtClient_" + QString::number(QRandomGenerator::global()->generate());
    client->setClientId(clientId);

    // 연결 성공 시
    connect(client, &QMqttClient::connected, this, [this]() {
        qDebug() << "✅ MQTT 연결 성공!";
        client->subscribe(QMqttTopicFilter("ajou/mqtttest/value"));
        client->subscribe(QMqttTopicFilter("dashboard/result/llm_summary"));
    });

    connect(client, &QMqttClient::disconnected, this, &Backend::onMqttDisconnected);
    connect(client, &QMqttClient::messageReceived, this, &Backend::onMqttMessageReceived);

    client->connectToHost();
}

// ----------------------
// MQTT 메시지 처리
// ----------------------
void Backend::onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic)
{
    QString val = QString::fromUtf8(message);

    if (topic.name() == "ajou/mqtttest/value") {
        emit newMqttValue(val);
        emit gauge1ValueChanged(val.toDouble());
    }
    else if (topic.name() == "dashboard/result/llm_summary") {
        qDebug() << "🎉 LLM 요약 수신:" << val;
        emit llmSummaryReady(val);
    }
}

// ----------------------
// MQTT 연결 끊김 처리
// ----------------------
void Backend::onMqttDisconnected()
{
    qDebug() << "❌ MQTT 연결 끊김 → 5초 후 재연결";
    QTimer::singleShot(5000, this, [this]() {
        client->connectToHost();
    });
}

// ----------------------
// MQTT 음성 명령 발행
// ----------------------
void Backend::startVoiceCommand()
{
    if (client->state() == QMqttClient::Connected) {
        client->publish(QMqttTopicName("dashboard/command/start_stt"), QByteArray("START"));
    }
}

// ----------------------
// 값 업데이트
// ----------------------
void Backend::updateData(double newValue)
{
    emit gauge1ValueChanged(newValue);
}

// ----------------------
// 앱 재시작
// ----------------------
void Backend::relaunchApp()
{
    QString program = QCoreApplication::applicationFilePath();
    QProcess::startDetached(program, {});
    QCoreApplication::quit();
}

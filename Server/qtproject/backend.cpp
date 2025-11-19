#include "backend.h"
#include <QDebug>
#include <QtMqtt/QMqttTopicFilter>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QTimer>
#include <QDir>
#include <QCoreApplication>
#include <QProcess>

// Backend 클래스
Backend::Backend(QObject *parent)
    : QObject(parent)
{
    loadAccountsFromFile();   // ✅ 프로그램 시작 시 로그인 정보 로드

    // ✅ 1초마다 현재시간 자동 갱신
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        m_currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"); // 시스템의 현재 시간을 알아내기
        qDebug() << "[TIME]" << m_currentTime;
        emit currentTimeChanged();
    });
    timer->start(1000);
}

// 로그인 시도 시 신호를 생성하는 코드
void Backend::login(const QString &id, const QString &pw)
{
    bool success = (accounts.contains(id) && accounts.value(id) == pw);
    writeLoginLog(id, pw, success);

    if (accounts.contains(id) && accounts.value(id) == pw) { // 로그인 성공
        emit loginSuccess();
    } else { // 로그인 실패
        emit loginFailed();
    }
}

// 계정 로드
void Backend::loadAccountsFromFile()
{
    qDebug() << "현재 작업 경로:" << QDir::currentPath();

    // 계정 json 파일 열기
    QFile file("accounts.json");
    // 로드 실패
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

    for (int i = 0; i < arr.size(); ++i) {
        const QJsonValue v = arr.at(i);
        if (!v.isObject()) continue;

        QJsonObject o = v.toObject();
        QString id = o["id"].toString();
        QString pw = o["pw"].toString();

        if (!id.isEmpty())
            accounts.insert(id, pw);
    }

    /*for (const QJsonValue &v : arr) {
        if (!v.isObject()) continue;
        QJsonObject o = v.toObject();
        QString id = o["id"].toString(); // id 정보 가져오기
        QString pw = o["pw"].toString(); // pw 정보 가져오기
        if (!id.isEmpty())
            accounts.insert(id, pw);
    }*/

    // qDebug() << "✅ 계정 로드 완료, 총" << accounts.size() << "개";
}

// 로그인 정보 기록
void Backend::writeLoginLog(const QString &id, const QString &pw, bool success)
{
    QString logPath = QCoreApplication::applicationDirPath() + "/../../log/logins.json";
    QFile file(logPath);

    QJsonArray logArray;

    // 1) 파일이 이미 존재하면 기존 내용 불러오기
    if (file.exists()) {
        if (file.open(QIODevice::ReadOnly)) {
            QByteArray data = file.readAll();
            file.close();

            QJsonDocument doc = QJsonDocument::fromJson(data);
            if (doc.isArray()) {
                logArray = doc.array();
            }
        }
    }

    // 2) 새로운 로그 객체 추가
    QJsonObject logEntry;
    logEntry["timestamp"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    logEntry["id"] = id;
    logEntry["pw"] = pw;
    logEntry["success"] = success;

    logArray.append(logEntry);

    // 3) 전체 로그 다시 저장 (덮어쓰기)
    if (file.open(QIODevice::WriteOnly)) {
        QJsonDocument saveDoc(logArray);
        file.write(saveDoc.toJson(QJsonDocument::Indented));
        file.close();
    }
}


void Backend::updateData(double newValue)
{
    // qDebug() << "새로운 데이터 수신:" << newValue;

    // QML에 실시간으로 값 전달
    emit gauge1ValueChanged(newValue);
}

// mqtt 구독 코드
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

void Backend::relaunchApp()
{
    QString program = QCoreApplication::applicationFilePath();
    QStringList args;

    QProcess::startDetached(program, args);
    QCoreApplication::quit();
}

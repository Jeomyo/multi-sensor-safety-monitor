#include "LoginManager.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QCoreApplication>
#include <QDir>
#include <QDateTime>

LoginManager::LoginManager(QObject *parent)
    : QObject(parent)
{
    loadUsersFromJson();   // 생성 시 JSON 로드
}

void LoginManager::loadUsersFromJson()
{
    QString appPath = QCoreApplication::applicationDirPath();
    QDir dir(appPath);

    dir.cdUp();
    dir.cdUp();

    QString jsonPath = dir.filePath("users.json");

    qDebug() << "🔎 JSON 로드 경로 =" << jsonPath;

    QFile file(jsonPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "⚠️ users.json 파일을 열 수 없습니다:" << jsonPath;
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        qWarning() << "⚠️ users.json JSON 포맷 오류";
        return;
    }

    QJsonObject root = doc.object();
    m_users = root["users"].toArray();

    qDebug() << "✅ JSON 사용자 수:" << m_users.size();
}

void LoginManager::addLoginRecord(const QString &id)
{
    // 실행 파일 경로 얻기
    QString appPath = QCoreApplication::applicationDirPath();
    QDir dir(appPath);

    // ➜ exe/debug → build → project_QML_1 이동 (2단계 위로)
    dir.cdUp();
    dir.cdUp();

    // ➜ log 폴더 생성
    if (!dir.exists("log")) {
        dir.mkdir("log");
    }

    dir.cd("log");   // log 폴더로 이동

    QString logPath = dir.filePath("logins.json");

    QFile file(logPath);
    QJsonArray logArray;

    // 기존 log 파일 읽기
    if (file.exists()) {
        if (file.open(QIODevice::ReadOnly)) {
            QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
            file.close();

            if (doc.isObject()) {
                logArray = doc.object()["logins"].toArray();
            }
        }
    }

    // 새로운 로그인 기록
    QJsonObject record;
    record["user"] = id;
    record["time"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    logArray.append(record);

    // 루트 객체 구성
    QJsonObject root;
    root["logins"] = logArray;

    QJsonDocument saveDoc(root);

    // 저장
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.write(saveDoc.toJson());
        file.close();
    }

    qDebug() << "📁 로그인 기록 저장:" << logPath;
}


bool LoginManager::login(const QString &id, const QString &pw)
{
    for (const QJsonValue &val : m_users) {
        QJsonObject user = val.toObject();

        QString uid = user["id"].toString();
        QString upw = user["pw"].toString();

        if (uid == id && upw == pw) {
            qDebug() << "🎉 로그인 성공:" << uid;

            addLoginRecord(id);   // ★ 로그인 기록 추가

            emit loginSucceeded();
            return true;
        }
    }

    qWarning() << "❌ 로그인 실패:" << id;
    emit loginFailed();
    return false;
}

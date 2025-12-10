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
    loadUsersFromJson();
}

void LoginManager::loadUsersFromJson()
{
    // ★ QRC prefix = "/icons"
    QFile file(":/icons/users.json");

    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "⚠️ 리소스 users.json 로드 실패!";
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
    if (m_users.isEmpty()) {
        qWarning() << "⚠️ 사용자 데이터가 없습니다!";
    }
}


// 로그인 기록은 로컬 파일로 저장
void LoginManager::addLoginRecord(const QString &id)
{
    QString appPath = QCoreApplication::applicationDirPath();
    QDir dir(appPath);

    // Debug → build → project_QML_1 (2단계 위)
    dir.cdUp();
    dir.cdUp();

    if (!dir.exists("log")) {
        dir.mkdir("log");
    }

    dir.cd("log");

    QString logPath = dir.filePath("logins.json");

    QFile file(logPath);
    QJsonArray logArray;

    if (file.exists() && file.open(QIODevice::ReadOnly)) {
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();

        if (doc.isObject()) {
            logArray = doc.object()["logins"].toArray();
        }
    }

    // 새로운 기록
    QJsonObject record;
    record["user"] = id;
    record["time"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    logArray.append(record);

    QJsonObject root;
    root["logins"] = logArray;

    QJsonDocument saveDoc(root);

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

        if (user["id"].toString() == id &&
            user["pw"].toString() == pw) {

            qDebug() << "🎉 로그인 성공:" << id;
            addLoginRecord(id);

            emit loginSucceeded();
            return true;
        }
    }

    qWarning() << "❌ 로그인 실패:" << id;
    emit loginFailed();
    return false;
}

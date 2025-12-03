#ifndef LOGINMANAGER_H
#define LOGINMANAGER_H

#include <QObject>
#include <QString>
#include <QJsonArray>

class LoginManager : public QObject {
    Q_OBJECT

public:
    explicit LoginManager(QObject *parent = nullptr);

    Q_INVOKABLE bool login(const QString &id, const QString &pw);

signals:
    void loginSucceeded();
    void loginFailed();

private:
    QJsonArray m_users;   // JSON 사용자 목록 저장
    void loadUsersFromJson();   // JSON 파일 읽기
    void addLoginRecord(const QString &id);
};

#endif // LOGINMANAGER_H

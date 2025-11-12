#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QtMqtt/QMqttClient>
#include <QMap>

class Backend : public QObject
{
    Q_OBJECT
public:
    explicit Backend(QObject *parent = nullptr);

    Q_INVOKABLE void login(const QString &id, const QString &pw); // 로그인용

    Q_INVOKABLE void updateData(double newValue); // 테스트용
    void setupMqtt();

private:
    void loadAccountsFromFile();
    QMap<QString, QString> accounts;
    QMqttClient *client = nullptr;

signals:
    void loginSuccess(); // 로그인 성공 신호
    void loginFailed(); // 로그인 실패 신호

    void gauge1ValueChanged(double value);
    void newMqttValue(QString value);
};

#endif // BACKEND_H

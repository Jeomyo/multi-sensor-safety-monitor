#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>

class Backend : public QObject
{
    Q_OBJECT
public:
    explicit Backend(QObject *parent = nullptr);

    Q_INVOKABLE void login(const QString &id, const QString &pw);

signals:
    void loginSuccess();
    void loginFailed();
};

#endif // BACKEND_H

#include "backend.h"
#include <QDebug>

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

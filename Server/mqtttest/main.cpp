#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "backend.h"

using namespace Qt::StringLiterals;

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv); // gui 초기화
    QQmlApplicationEngine engine; // qml파일 관리 객체

    Backend backend; // 직접 만든 클래스
    backend.setupMqtt();

    engine.rootContext()->setContextProperty("backend", &backend); // backend 이름으로 객체에 접근가능하게 하는 것

    // ✅ QRC 경로 수정 (핵심)
    const QUrl url(u"qrc:mqtttest/qml/AppWindow.qml"_s);

    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.load(url);
    return app.exec();
}

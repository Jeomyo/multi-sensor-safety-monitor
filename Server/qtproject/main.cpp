#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "backend.h"

using namespace Qt::StringLiterals;

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv); // gui 초기화
    QQmlApplicationEngine engine; // qml파일 관리 객체

    Backend backend; // 데이터를 관리하는 클래스
    backend.setupMqtt();

    engine.rootContext()->setContextProperty("backend", &backend); // backend 이름으로 객체에 접근가능하게 하는 것

    const QUrl url(u"qrc:qtproject/qml/AppWindow.qml"_s); // AppWindow.qml 실행

    QObject::connect(
        &engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.load(url);
    return app.exec();
}

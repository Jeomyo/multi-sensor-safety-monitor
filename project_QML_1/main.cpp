#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickItem>
#include <QVideoSink>
#include <QMediaPlayer>
#include "sensor_data_provider.h"
#include "SystemMonitor.h"
#include "LoginManager.h"

SystemMonitor* g_systemMonitor = nullptr;

int main(int argc, char *argv[])
{

    // 🔥 FFmpeg / swscaler stderr 로그 완전 차단
    //freopen("NUL", "w", stderr);

    //qputenv("QT_LOGGING_RULES", "*.debug=false;qt.multimedia.ffmpeg.*=false");
    qputenv("QT_FFMPEG_LOG_LEVEL", "quiet");

    // ✅ 로컬 파일 XMLHttpRequest 허용
    qputenv("QML_XHR_ALLOW_FILE_READ", QByteArray("1"));
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
    qputenv("QT_DEBUG_PLUGINS", QByteArray("1"));

    //GStreamer low-latency 설정
    qputenv("GST_RTSP_LATENCY", "30");          // 버퍼 지연 최소화
    qputenv("GST_RTSPSRC_PROTOCOLS", "tcp");    // TCP 사용
    qputenv("GST_DEBUG", "2");                  // 디버그 로그
    qputenv("GST_PLAY_FLAGS", "0x17");          // 오디오+비디오+텍스트
    qputenv("GST_PLUGIN_FEATURE_RANK", "0");
    qputenv("GST_DEBUG_DUMP_DOT_DIR", ".");





    // ⚙️ 필요 시 추가 옵션 예시:
    // qputenv("GST_RTSPSRC_PROTOCOLS", "udp");  // TCP 대신 UDP 사용
    // qputenv("GST_RTSPSRC_DO_RTSP_KEEP_ALIVE", "true"); // 세션 유지


    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // ✅ 객체 생명주기 고정
    static SensorDataProvider sensorProvider;
    static SystemMonitor systemMonitor;
    static LoginManager loginManager;
    g_systemMonitor = &systemMonitor;

    // ✅ QML 등록
    engine.rootContext()->setContextProperty("sensorProvider", &sensorProvider);
    engine.rootContext()->setContextProperty("systemMonitor", &systemMonitor);
    engine.rootContext()->setContextProperty("loginManager", &loginManager);

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app,
                     []() { QCoreApplication::exit(-1); },
                     Qt::QueuedConnection);

    // ✅ MQTT 연결
    //sensorProvider.connectToBroker("broker.hivemq.com", 1883);
    sensorProvider.connectToBroker("192.168.0.74", 1883);

    // ✅ QML 로드
    engine.loadFromModule("project_QML_1", "Main");

    // ✅ [추가] QML에서 MediaPlayer 찾아서 RTSP FPS 연결
    /*
    if (!engine.rootObjects().isEmpty()) {
        QObject* root = engine.rootObjects().first();

        // StackLayout 안의 Streaming 페이지 접근
        QObject* streamingPage = root->findChild<QObject*>("streamingPage");
        if (streamingPage) {
            QObject* playerObj = streamingPage->findChild<QObject*>("player");
            if (playerObj) {
                // MediaPlayer의 videoSink() 속성을 가져오기
                QVariant sinkVar = playerObj->property("videoSink");
                if (sinkVar.isValid()) {
                    QVideoSink* sink = qvariant_cast<QVideoSink*>(sinkVar);
                    if (sink) {
                        systemMonitor.attachVideoSink(sink);
                        qDebug() << "✅ RTSP FPS 모니터링 연결 완료";
                    } else {
                        qWarning() << "⚠️ QVideoSink 캐스팅 실패";
                    }
                } else {
                    qWarning() << "⚠️ videoSink 속성 접근 실패";
                }
            } else {
                qWarning() << "⚠️ Streaming.qml의 player 객체를 찾지 못함";
            }
        } else {
            qWarning() << "⚠️ streamingPage 객체를 찾지 못함";
        }
    }
    */

    return app.exec();
}

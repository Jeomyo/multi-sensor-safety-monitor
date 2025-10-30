#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMap>                // 작업자 관리를 위해 추가
#include <QStringList>         // 데이터 기록을 위해 추가
#include <QtMqtt/QMqttClient>  // MQTT 사용을 위해 추가
#include <QGraphicsScene>      // 그래픽 Scene 사용을 위해 추가
#include <QGraphicsEllipseItem>// 작업자 아이템 사용을 위해 추가
#include "websocketserver.h"   // WebSocket 서버 헤더 추가

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // MQTT 메시지가 도착하면 호출될 슬롯
    void onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic);
    // '상태 요약' 버튼이 클릭되면 호출될 슬롯
    void on_summarizeButton_clicked();
    // WebSocket 서버로부터 명령을 받으면 호출될 슬롯
    void handleCommand(const QString &commandJson);

private:
    Ui::MainWindow *ui;
    QGraphicsScene *m_scene;                     // 그래픽 도화지
    QMqttClient *m_mqttClient;                   // MQTT 클라이언트
    WebSocketServer *m_webSocketServer;          // WebSocket 서버
    QStringList m_sensorDataHistory;           // 센서 데이터 기록
    QMap<QString, QGraphicsEllipseItem*> m_workers; // 작업자 아이템 관리 (ID, 아이템 포인터)
};
#endif // MAINWINDOW_H

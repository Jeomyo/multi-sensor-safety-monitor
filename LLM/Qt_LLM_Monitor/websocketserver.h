#ifndef WEBSOCKETSERVER_H
#define WEBSOCKETSERVER_H

#include <QObject>
#include <QtWebSockets/QWebSocketServer> // WebSocket 서버 헤더
#include <QtWebSockets/QWebSocket>   // WebSocket 클라이언트 헤더

class WebSocketServer : public QObject
{
    Q_OBJECT
public:
    explicit WebSocketServer(quint16 port, QObject *parent = nullptr);
    ~WebSocketServer() override;

signals:
    // JSON 명령을 받으면 MainWindow로 전달할 시그널
    void commandReceived(const QString &commandJson);

private slots:
    // 새 클라이언트가 접속했을 때 호출될 슬롯
    void onNewConnection();
    // 클라이언트가 메시지를 보냈을 때 호출될 슬롯
    void processTextMessage(QString message);
    // 클라이언트 접속이 끊겼을 때 호출될 슬롯
    void socketDisconnected();

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
};

#endif // WEBSOCKETSERVER_H

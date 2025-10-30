#include "websocketserver.h"
#include <QDebug>

WebSocketServer::WebSocketServer(quint16 port, QObject *parent) : QObject(parent)
{
    // WebSocket 서버 생성 및 리슨 시작
    m_pWebSocketServer = new QWebSocketServer(QStringLiteral("Command Server"), QWebSocketServer::NonSecureMode, this);
    if (m_pWebSocketServer->listen(QHostAddress::Any, port)) {
        qDebug() << "WebSocket server listening on port" << port;
        // 새 클라이언트 연결 시그널과 슬롯 연결
        connect(m_pWebSocketServer, &QWebSocketServer::newConnection, this, &WebSocketServer::onNewConnection);
    } else {
        qDebug() << "Failed to start WebSocket server on port" << port;
    }
}

WebSocketServer::~WebSocketServer()
{
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

void WebSocketServer::onNewConnection()
{
    // 새로 접속한 클라이언트 소켓 가져오기
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();
    qDebug() << "Client connected:" << pSocket->peerAddress().toString();

    // 클라이언트가 메시지를 보내거나 연결 끊을 때 처리할 슬롯 연결
    connect(pSocket, &QWebSocket::textMessageReceived, this, &WebSocketServer::processTextMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &WebSocketServer::socketDisconnected);

    // 클라이언트 목록에 추가
    m_clients << pSocket;
}

void WebSocketServer::processTextMessage(QString message)
{
    qDebug() << "Command received:" << message;
    // 받은 JSON 명령을 MainWindow로 전달하는 시그널 발생
    emit commandReceived(message);
}

void WebSocketServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    qDebug() << "Client disconnected:" << pClient->peerAddress().toString();
    if (pClient) {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

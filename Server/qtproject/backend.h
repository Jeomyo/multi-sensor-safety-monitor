#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>
#include <QString>
#include <QtMqtt/QMqttClient>
#include <QMap>

class Backend : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString currentTime READ currentTime NOTIFY currentTimeChanged) // 현재 시간 데이터 받아오기

public:
    explicit Backend(QObject *parent = nullptr); // main.cpp에서 사용하는 Backend 클래스
    Q_INVOKABLE void login(const QString &id, const QString &pw); // 로그인용
    Q_INVOKABLE void updateData(double newValue); // mqtt 테스트용
    Q_INVOKABLE void startVoiceCommand(); // 음성 명령 시작

    void setupMqtt(); // mqtt 설정 함수
    QString currentTime() const { return m_currentTime; }
    Q_INVOKABLE void relaunchApp(); // 재실행용 함수

private slots:
    // MQTT 메시지 수신 슬롯
    void onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic);
    // MQTT 연결 끊김 처리 슬롯
    void onMqttDisconnected();

private:
    void loadAccountsFromFile();
    void writeLoginLog(const QString &id, const QString &pw, bool success);
    QMap<QString, QString> accounts;
    QMqttClient *client = nullptr; // mqtt 클라이언트
    QString m_currentTime; // 현재 시간 저장 변수

signals:
    void loginSuccess(); // 로그인 성공 신호
    void loginFailed(); // 로그인 실패 신호

    void gauge1ValueChanged(double value); // 게이지 값 신호
    void newMqttValue(QString value); // 받은 mqtt 값 신호

    void currentTimeChanged(); // 시간 바뀌었다는 신호

    void llmSummaryReady(const QString &summaryText);
};

#endif // BACKEND_H

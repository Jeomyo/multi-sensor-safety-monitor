#ifndef SENSOR_DATA_PROVIDER_H
#define SENSOR_DATA_PROVIDER_H

#include <QObject>
#include <QtMqtt/QMqttClient>

// ✅ 추가: JSON 처리를 위한 헤더
#include <QJsonArray>

class SensorDataProvider : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double temperature READ temperature NOTIFY temperatureChanged)
    Q_PROPERTY(double humidity READ humidity NOTIFY humidityChanged)
    Q_PROPERTY(double dust READ dust NOTIFY dustChanged)

public:
    explicit SensorDataProvider(QObject *parent = nullptr);

    double temperature() const { return m_temperature; }
    double humidity() const { return m_humidity; }
    double dust() const { return m_dust; }

    Q_INVOKABLE void connectToBroker(const QString &host, int port);

signals:
    // ✅ 기존 시그널 (그대로 유지)
    void temperatureChanged();
    void humidityChanged();
    void dustChanged();

    // ✅ 추가 시그널 (지도, 장애물, 작업자용)
    void mapUpdated(QJsonArray mapPoints);
    void obstaclesUpdated(QJsonArray obstacles);
    void workersUpdated(QJsonArray workers);

    // ✅ 추가 시그널 (선민이용)
    void workStartModeChanged(int value);
    void workEndModeChanged(int value);
    void systemMsgOneReceived(QString msg);

private slots:
    void handleMessage(const QByteArray &message, const QMqttTopicName &topic);

private:
    QMqttClient *m_client;
    double m_temperature = 0.0;
    double m_humidity = 0.0;
    double m_dust = 0.0;

    // ✅ 추가: 추후 필요 시 내부 저장용 멤버 (선택적)
    // QList<QJsonObject> m_mapPoints;
    // QList<QJsonObject> m_obstacles;
    // QList<QJsonObject> m_workers;
};

#endif // SENSOR_DATA_PROVIDER_H

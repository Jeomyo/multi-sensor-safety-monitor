#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#include <QObject>
#include <QTimer>
#include <QProcess>

class QVideoSink;
class QVideoFrame;

class SystemMonitor : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double cpuUsage READ cpuUsage NOTIFY cpuUsageChanged)
    Q_PROPERTY(double gpuUsage READ gpuUsage NOTIFY gpuUsageChanged)
    Q_PROPERTY(int rtspFps READ rtspFps NOTIFY rtspFpsChanged)
    Q_PROPERTY(double mqttLatency READ mqttLatency NOTIFY mqttLatencyChanged)

public:
    explicit SystemMonitor(QObject* parent = nullptr);

    double cpuUsage() const { return m_cpuUsage; }
    double gpuUsage() const { return m_gpuUsage; }
    int rtspFps() const { return m_rtspFps; }
    double mqttLatency() const { return m_mqttLatency; }

    void attachVideoSink(QVideoSink* sink);
    void updateMqttLatency(double latencyMs);

signals:
    void cpuUsageChanged();
    void gpuUsageChanged();
    void rtspFpsChanged();
    void mqttLatencyChanged();

private slots:
    void updateMetrics();

private:
    QTimer m_timer;

    // CPU
    double m_cpuUsage = 0.0;

    // GPU (QProcess 멤버로 유지)
    QProcess* m_gpuProcess;
    double m_gpuUsage = 0.0;

    // RTSP FPS
    QTimer m_fpsTimer;
    int m_frameCount = 0;
    int m_rtspFps = 0;

    // MQTT Latency
    double m_mqttLatency = 0.0;
};

#endif // SYSTEMMONITOR_H

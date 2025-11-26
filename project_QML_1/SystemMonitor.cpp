#include "SystemMonitor.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QDateTime>
#include <QElapsedTimer>

#include <QVideoSink>
#include <QVideoFrame>

SystemMonitor::SystemMonitor(QObject* parent)
    : QObject(parent)
    , m_gpuProcess(new QProcess(this))
{
    // CPU/GPU 타이머
    connect(&m_timer, &QTimer::timeout, this, &SystemMonitor::updateMetrics);
    m_timer.start(1000);

    // FPS 타이머
    connect(&m_fpsTimer, &QTimer::timeout, this, [this]() {
        m_rtspFps = m_frameCount;
        m_frameCount = 0;
        emit rtspFpsChanged();
    });
    m_fpsTimer.start(1000);
}

/* --------------------------------------------------------
 * 시스템 모니터링 업데이트
 * -------------------------------------------------------- */
void SystemMonitor::updateMetrics()
{
    /* -------------------------------
     * 1) CPU 사용률 (Linux /proc/stat)
     * ------------------------------- */
    QFile file("/proc/stat");
    if (file.open(QIODevice::ReadOnly)) {
        QByteArray line = file.readLine();
        QList<QByteArray> v = line.split(' ');
        if (v.size() > 8) {
            static quint64 prevIdle = 0, prevTotal = 0;
            quint64 user = v[2].toUInt(), nice = v[3].toUInt();
            quint64 system = v[4].toUInt(), idle = v[5].toUInt();

            quint64 total = user + nice + system + idle;
            quint64 totalDiff = total - prevTotal;
            quint64 idleDiff = idle - prevIdle;

            if (totalDiff > 0) {
                m_cpuUsage = 100.0 * (1.0 - (double)idleDiff / totalDiff);
                emit cpuUsageChanged();
            }

            prevIdle = idle;
            prevTotal = total;
        }
    }

    /* ----------------------------------------------------
     * 2) GPU 사용률
     *    - Raspberry Pi: nvidia-smi 없음 → 건너뜀
     *    - Windows/Linux + NVIDIA: 정상 측정
     * ---------------------------------------------------- */

    // Nvidia GPU 존재 확인
    bool hasNvidia = QFile::exists("/usr/bin/nvidia-smi")
                     || QFile::exists("C:/Windows/System32/nvidia-smi.exe");

    if (!hasNvidia) {
        m_gpuUsage = 0.0;     // Pi에서는 GPU 사용률 항상 0
        emit gpuUsageChanged();
        return;
    }

    // 명령 실행
#ifdef Q_OS_WIN
    QString cmd = "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits";
    m_gpuProcess->start("cmd", {"/C", cmd});
#else
    QString cmd = "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits";
    m_gpuProcess->start("bash", {"-c", cmd});
#endif

    if (!m_gpuProcess->waitForFinished(800)) {     // 0.8초 대기
        m_gpuProcess->kill();                      // 종료 보장
        m_gpuProcess->waitForFinished();
        return;
    }

    QString output = m_gpuProcess->readAllStandardOutput().trimmed();
    if (!output.isEmpty()) {
        m_gpuUsage = output.toDouble();
        emit gpuUsageChanged();
    }
}

/* --------------------------------------------------------
 * FPS 측정
 * -------------------------------------------------------- */
void SystemMonitor::attachVideoSink(QVideoSink* sink)
{
    if (!sink) return;

    connect(sink, &QVideoSink::videoFrameChanged, this,
            [this](const QVideoFrame&) {
                m_frameCount++;
            });
}

/* --------------------------------------------------------
 * MQTT 지연 갱신
 * -------------------------------------------------------- */
void SystemMonitor::updateMqttLatency(double latencyMs)
{
    m_mqttLatency = latencyMs;
    emit mqttLatencyChanged();
}

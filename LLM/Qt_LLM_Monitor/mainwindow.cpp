#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QGraphicsPixmapItem> // 지도 이미지 표시
#include <QJsonDocument>       // JSON 파싱
#include <QJsonObject>         // JSON 파싱
#include <QJsonValue>          // JSON 파싱
#include <QJsonParseError>     // JSON 에러 처리
#include <QDebug>              // 디버깅 메시지
#include <QProcess>            // 외부 스크립트 실행
#include <QMessageBox>         // 팝업 메시지

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("멀티센서 안전 모니터링 대시보드");

    // --- 그래픽 설정 ---
    m_scene = new QGraphicsScene(this);
    ui->g->setScene(m_scene); // UI의 graphicsView와 연결

    // 지도 배경 이미지 로드 및 추가 (리소스 파일 사용)
    QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem(QPixmap(":/lidar_map.png"));
    if (pixmapItem->pixmap().isNull()) {
        qWarning() << "Failed to load map image ':/lidar_map.png'. Make sure it's added to resources.qrc";
    } else {
        m_scene->addItem(pixmapItem);
    }


    // --- MQTT 설정 ---
    m_mqttClient = new QMqttClient(this);
    m_mqttClient->setHostname("localhost"); // MQTT 브로커 주소
    m_mqttClient->setPort(1883);

    // MQTT 메시지 도착 시그널 연결
    connect(m_mqttClient, &QMqttClient::messageReceived, this, &MainWindow::onMqttMessageReceived);

    // MQTT 브로커 연결 시도
    m_mqttClient->connectToHost();

    // MQTT 연결 성공 시 토픽 구독
    connect(m_mqttClient, &QMqttClient::connected, [this]() {
        qDebug() << "MQTT Broker Connected!";
        // 두 개의 토픽을 구독합니다.
        auto subSensors = m_mqttClient->subscribe(QString("factory/sensors")); // 설비 센서 데이터
        if (!subSensors) {
            qWarning() << "Could not subscribe to factory/sensors. Is the broker running?";
        }
        auto subWorkers = m_mqttClient->subscribe(QString("factory/safety")); // 작업자 위치/상태 데이터
        if (!subWorkers) {
            qWarning() << "Could not subscribe to factory/safety. Is the broker running?";
        }
    });

    // --- WebSocket 서버 설정 ---
    quint16 wsPort = 12345; // 파이썬 음성인식 스크립트와 통신할 포트
    m_webSocketServer = new WebSocketServer(wsPort, this);
    // WebSocket 서버가 명령 수신 시 시그널 연결
    connect(m_webSocketServer, &WebSocketServer::commandReceived, this, &MainWindow::handleCommand);

    // --- UI 버튼 시그널 연결 ---
    // '상태 요약' 버튼 클릭 시 on_summarizeButton_clicked 슬롯 호출
    connect(ui->summarizeButton, &QPushButton::clicked, this, &MainWindow::on_summarizeButton_clicked);
}

MainWindow::~MainWindow()
{
    delete ui;
    // MQTT 클라이언트 연결 종료 등 리소스 정리 코드가 필요하다면 여기에 추가
}

// MQTT 메시지 처리 함수
void MainWindow::onMqttMessageReceived(const QByteArray &message, const QMqttTopicName &topic)
{
    qDebug() << "Message received on topic:" << topic.name() << "Data:" << message;

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(message, &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "Failed to parse MQTT JSON:" << parseError.errorString() << "Original data:" << message;
        return;
    }
    QJsonObject data = doc.object();

    // 토픽 이름으로 어떤 종류의 데이터인지 구분
    if (topic.name() == "factory/sensors") {
        // 설비 센서 데이터 처리 (온도, 진동 등)
        if (data.contains("temperature") && data.contains("vibration")) {
            double temp = data["temperature"].toDouble();
            double vib = data["vibration"].toDouble();
            ui->tempLabel->setText(QString("온도: %1 °C").arg(temp, 0, 'f', 2));
            ui->vibLabel->setText(QString("진동: %1").arg(vib, 0, 'f', 2));
            m_sensorDataHistory.append(QString(message)); // 요약을 위해 기록
        }
    } else if (topic.name() == "factory/safety") {
        // 작업자 데이터 처리 (위치, 상태 등)
        if (data.contains("worker_id") && data.contains("position")) {
            QString workerId = data["worker_id"].toString();
            double x = data["position"].toObject()["x"].toDouble();
            double y = data["position"].toObject()["y"].toDouble();

            // 작업자 아이템 관리 (없으면 생성, 있으면 위치 업데이트)
            if (!m_workers.contains(workerId)) {
                QGraphicsEllipseItem *worker = new QGraphicsEllipseItem(0, 0, 20, 20); // 원 크기 20x20
                worker->setBrush(Qt::green); // 기본 색상 녹색
                m_scene->addItem(worker);
                m_workers.insert(workerId, worker); // 맵에 추가
                qDebug() << "New worker added:" << workerId;
            }
            m_workers[workerId]->setPos(x * 100, y * 100); // 위치 업데이트 (좌표 스케일 조정 필요)

            // (추가 확장 가능) 헬멧 착용 여부 등에 따라 색상 변경 로직 추가

        }
    }
}

// '상태 요약' 버튼 클릭 시 실행될 함수
void MainWindow::on_summarizeButton_clicked()
{
    QString dataString = m_sensorDataHistory.join("\n");
    if (dataString.isEmpty()) {
        QMessageBox::information(this, "요약", "요약할 데이터가 없습니다.");
        return;
    }

    QProcess *process = new QProcess(this);
    QString pythonExecutable = "C:/Users/ldi03/OneDrive/바탕 화면/whisper-env/Scripts/python.exe"; // 가상환경 파이썬 경로
    QStringList arguments;
    arguments << "C:/Users/ldi03/OneDrive/바탕 화면/summarizer.py" << dataString; // 스크립트 경로

    connect(process, &QProcess::finished, [=](int exitCode){
        if (exitCode == 0) {
            QString summary = process->readAllStandardOutput();
            QMessageBox::information(this, "상태 요약", summary);
        } else {
            QString error = process->readAllStandardError();
            QMessageBox::warning(this, "오류", "요약에 실패했습니다:\n" + error);
        }
        process->deleteLater(); // 프로세스 객체 메모리 해제
    });

    process->start(pythonExecutable, arguments);
}

// WebSocket으로 받은 명령을 처리하는 함수
void MainWindow::handleCommand(const QString &commandJson)
{
    qDebug() << "Command received via WebSocket:" << commandJson;
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(commandJson.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "Failed to parse command JSON:" << parseError.errorString() << "Original data:" << commandJson;
        return;
    }
    QJsonObject command = doc.object();

    if (command.contains("action") && command["action"].toString() == "track_worker") {
        QString targetId = command["target"].toString();
        qDebug() << "Executing command: Track worker" << targetId;

        // 모든 작업자 색상을 기본(녹색)으로 변경
        for (QGraphicsEllipseItem *worker : m_workers.values()) {
            worker->setBrush(Qt::green);
        }
        // 타겟 작업자만 청록색으로 변경
        if (m_workers.contains(targetId)) {
            m_workers[targetId]->setBrush(Qt::cyan);
            qDebug() << "Worker" << targetId << "highlighted.";
        } else {
            qWarning() << "Worker" << targetId << "not found for tracking.";
        }
    }
    // 다른 action들에 대한 처리 로직 추가 가능 (예: 화면 이동, 확대 등)
}

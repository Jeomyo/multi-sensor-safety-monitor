import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1600
    height: 900
    title: "Digital Twin Safety Monitoring"
    // 1) Worker 수 저장 변수
     property int workerCount: 0

     // 2) MQTT worker 업데이트를 직접 받는 부분
     Connections {
         target: sensorProvider

         function onWorkersUpdated(workers) {
         root.workerCount = workers.length // 총 작업자 수 업데이트
         }
     }

    Rectangle {
        anchors.fill: parent
        color: "#101214"
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        // ===========================
        // 상단 — RTSP 영역
        // ===========================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: root.height * 0.15
            Layout.topMargin: 10
            spacing: 6

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 5

                Item { Layout.fillWidth: true }

                Rectangle {
                    Layout.preferredWidth: root.width * 0.14
                    Layout.fillHeight: true
                    radius: 6
                    color: "#101214"
                    border.color: "#333"
                    border.width: 0

                    Streaming {
                        anchors.fill: parent
                        rtspEnable: true       // 집에서는 꺼두기
                        sourceUrl: "rtsp://admin:qw12qw12%21@192.168.0.64:554/Streaming/Channels/101"
                    }
                }

                Rectangle {
                    Layout.preferredWidth: root.width * 0.14
                    Layout.fillHeight: true
                    radius: 6
                    color: "#101214"
                    border.color: "#333"
                    border.width: 0

                    Streaming {
                        anchors.fill: parent
                        rtspEnable: false
                        sourceUrl: "rtsp://127.0.0.1:8554/stream"
                    }
                }

                Item { Layout.fillWidth: true }
            }

            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                spacing: 40

                Row {
                    spacing: 6

                    Text {
                        text: "SYSTEM ONLINE"
                        color: "#ccc"
                        font.pixelSize: 12
                    }

                    Item {
                        width: 12
                        height: 12

                        Rectangle {
                            anchors.fill: parent
                            radius: width / 2
                            color: "#4FC3F7"
                        }

                        Rectangle {
                            anchors.centerIn: parent
                            width: 12; height: 12
                            radius: width / 2
                            color: "#4FC3F7"
                            opacity: 0.4

                            SequentialAnimation on scale {
                                loops: Animation.Infinite
                                running: true
                                NumberAnimation { from: 1.0; to: 2.0; duration: 1200 }
                            }

                            SequentialAnimation on opacity {
                                loops: Animation.Infinite
                                running: true
                                NumberAnimation { from: 0.4; to: 0.0; duration: 1200 }
                            }
                        }
                    }
                }

                Text {
                    text: "DATA LATENCY: " + systemMonitor.mqttLatency.toFixed(1) + " ms"
                    color: "#ccc"
                    font.pixelSize: 12
                }

                Text {
                    text: "WORKERS MONITORED: " + root.workerCount + " / 6"
                    color: "#ccc"
                    font.pixelSize: 12
                }
            }
        }

        // ===========================
        // 메인 3단 구조
        // ===========================
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: root.height * 0.70
            spacing: 10

            // 좌측
            Rectangle {
                Layout.preferredWidth: root.width * 0.18
                Layout.fillHeight: true
                radius: 8
                color: "#1E1F22"
                border.color: "#0324fc"
                border.width: 1

                DashBoard {
                    anchors.fill: parent
                    anchors.margins: 12
                }
            }

            // 중앙
            Rectangle {
                Layout.preferredWidth: root.width * 0.60
                Layout.fillHeight: true
                radius: 8
                color: "#1E1F22"
                border.color: "#0324fc"
                border.width: 1

                MapPage_2 {
                    id: mapArea
                    anchors.fill: parent
                    anchors.margins: 12
                }
            }

            // 우측
            InsightsPanel {
                Layout.preferredWidth: root.width * 0.2
                Layout.fillHeight: true
            }
        }

        Item {
            Layout.fillWidth: true
            height: 10
            Layout.topMargin: 20

            Rectangle {
                width: parent.width * 0.95
                height: 1
                color: "#4FC3F7"
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }

        // ===========================
        // 하단 메뉴바
        // ===========================
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: root.height * 0.08

            BottomNavigationBar {
                anchors.fill: parent
            }
        }
    }
}

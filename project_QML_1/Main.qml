import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15


Item {
    id: root
    anchors.fill: parent

    Rectangle {
            anchors.fill: parent
            color: "#101214"
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        // ===========================
        // 상단 — CCTV 스트림 자리만 (RTSP 없음)
        // ===========================
        // ===========================
        // 상단 영역: RTSP 2개 + 상태바
        // ===========================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: parent.height * 0.15
            Layout.topMargin: 10
            spacing: 6

            // ---------------------------
            // RTSP 화면 2개 (중앙 정렬)
            // ---------------------------
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height * 0.15
                spacing: 5

                Item {
                    Layout.fillWidth: true
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
                        sourceUrl: "rtsp://127.0.0.1:8554/stream"
                    }
                }
                Item {
                        Layout.fillWidth: true
                    }
            }

            // ---------------------------
            // 시스템 상태바
            // ---------------------------
            // ---------------------------
            // 시스템 상태바 (Pulse 효과 포함)
            // ---------------------------
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height * 0.07
                spacing: 40

                // SYSTEM ONLINE + 초록 Pulse
                Row {
                    spacing: 6

                    Text {
                        id: statusText
                        text: "SYSTEM ONLINE"
                        color: "#ccc"
                        font.pixelSize: 12
                    }

                    // 🔥 Pulse 애니메이션 들어간 초록 상태 표시
                    Item {
                        width: 12
                        height: 12
                        anchors.verticalCenter: statusText.verticalCenter

                        // 실제 표시되는 원
                        Rectangle {
                            id: greenDot
                            width: parent.width
                            height: parent.height
                            radius: width / 2
                            color: "#4FC3F7"
                        }

                        // 퍼져나가는 pulse 효과
                        Rectangle {
                            id: greenPulse
                            anchors.centerIn: parent
                            width: 12
                            height: 12
                            radius: width / 2
                            color: "#4FC3F7"
                            opacity: 0.4

                            // 원이 커지는 애니메이션
                            SequentialAnimation on scale {
                                loops: Animation.Infinite
                                running: true
                                NumberAnimation { from: 1.0; to: 2.0; duration: 1200 }
                            }

                            // 바깥 원이 점점 사라지는 애니메이션
                            SequentialAnimation on opacity {
                                loops: Animation.Infinite
                                running: true
                                NumberAnimation { from: 0.4; to: 0.0; duration: 1200 }
                            }
                        }
                    }
                }

                // Latency 출력
                Text {
                    //text: "DATA LATENCY: 332 ms"
                    text: "DATA LATENCY: " + systemMonitor.mqttLatency.toFixed(1) + " ms"
                    color: "#ccc"
                    font.pixelSize: 12
                }

                // Worker 출력
                Text {
                    text: "WORKERS MONITORED: 2 / 6"
                    color: "#ccc"
                    font.pixelSize: 12
                }
            }

        }


        // ===========================
        // 메인 3단 구조
        // ===========================
        Row {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10
            // ===========================
            // 좌측 — 환경 센서 패널
            // ===========================
            Rectangle {
                width: parent.width * 0.18
                height: parent.height
                radius: 8
                color: "#1E1F22"
                border.color: "#0324fc"
                border.width: 1

                DashBoard {
                    anchors.fill: parent
                    anchors.margins: 12
                }
            }

            // ===========================
            // 중앙 — 맵 패널
            // ===========================
            Rectangle {
                width: parent.width * 0.6
                height: parent.height
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

            // ===========================
            // 우측 — AI Insights
            // ===========================
            Rectangle {
                width: parent.width * 0.2
                height: parent.height
                radius: 8
                color: "#1E1F22"
                border.color: "#0324fc"
                border.width: 1

                Column {
                    id: insightsPanel
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 10
                    property bool micActive: false
                    // 작업자 출근 퇴근
                    property string workState: "미정"

                    Connections {
                        target: sensorProvider

                        function onWorkStartModeChanged(value) {
                               insightsPanel.workState = "출근"
                           }
                        function onWorkEndModeChanged(value) {
                               insightsPanel.workState = "퇴근"
                           }
                    }
                    //////////////////////////////////////////////////////////////////////////

                    Text { text: "AI INSIGHTS"; color: "#4FC3F7"; font.pixelSize: 20 }

                    Rectangle { height: 1; width: parent.width; color: "#444" }

                    Text { text: "Latest Alerts Log"; color: "#fff"; font.pixelSize: 16 }

                    //Text {  text: "Safety Violations"; color: "#fff"; font.pixelSize: 16 }

                    Rectangle {
                        width: parent.width
                        height: 160
                        radius: 6
                        color: "#1E1F22"
                        border.color: "#333"
                        border.width: 0

                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.margins: 6

                        Column {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4

                            // 첫 줄: 빨간 점 + "ID: 003 (Kim)"
                            Row {
                                spacing: 6
                                anchors.left: parent.left

                                // 🔴 아이콘
                                Item {
                                    width: 12
                                    height: 12
                                    anchors.verticalCenter: parent.verticalCenter

                                    Rectangle {
                                        anchors.fill: parent
                                        radius: width / 2
                                        color: "red"
                                    }

                                    Rectangle {
                                        anchors.centerIn: parent
                                        width: 12
                                        height: 12
                                        radius: width / 2
                                        color: "red"
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

                                // 첫 줄 텍스트
                                Text {
                                    text: "ID: 7 (작업자B)"
                                    color: "#ccc"
                                    font.pixelSize: 14
                                    verticalAlignment: Text.AlignVCenter
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                            }

                            // 두 번째 줄: 사유
                            Text {
                                text: "    ▶ No Helmet"
                                color: "#ccc"
                                font.pixelSize: 14
                                anchors.left: parent.left
                            }



                            Row {
                                spacing: 6
                                anchors.left: parent.left

                                //
                                Item {
                                    width: 12
                                    height: 12
                                    anchors.verticalCenter: parent.verticalCenter

                                    Rectangle {
                                        anchors.fill: parent
                                        radius: width / 2
                                        color: "#4FC3F7"
                                    }

                                    Rectangle {
                                        anchors.centerIn: parent
                                        width: 12
                                        height: 12
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

                                // 첫 줄 텍스트
                                Text {
                                    text: "ID: 12 (작업자A)"
                                    color: "#ccc"
                                    font.pixelSize: 14
                                    verticalAlignment: Text.AlignVCenter
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                            }

                            // 두 번째 줄: 사유
                            Text {
                                text: "    ▶ " + insightsPanel.workState
                                color: "#ccc"
                                font.pixelSize: 14
                                anchors.left: parent.left
                            }
                        }
                    }





                    Rectangle { height: 1; width: parent.width; color: "#444" }

                    Text { text: "PPE Compliance"; color: "#fff"; font.pixelSize: 16 }

                    Repeater {
                        model: 3

                        Row {
                            spacing: 8

                            Text {
                                id: labelText
                                text: ["S", "H", "V"][index]
                                color: "#ccc"
                                font.pixelSize: 14
                            }

                            // index = 0 → S
                            Rectangle {
                                visible: index === 0
                                width: 80; height: 10; radius: 3
                                color: "#4CAF50"

                                anchors.verticalCenter: labelText.verticalCenter
                            }

                            // index = 1 → H
                            Rectangle {
                                visible: index === 1
                                width: 80; height: 10; radius: 3
                                color: "#FFC107"

                                anchors.verticalCenter: labelText.verticalCenter
                            }

                            // index = 2 → V
                            Rectangle {
                                visible: index === 2
                                width: 0; height: 10; radius: 3
                                color: "#F44336"

                                anchors.verticalCenter: labelText.verticalCenter
                            }
                        }
                    }



                    Rectangle { height: 1; width: parent.width; color: "#444" }
                    // ===============================
                    // 🎤 마이크 아이콘 (클릭 토글)
                    // ===============================
                    Item {
                        width: 80
                        height: 80
                        anchors.horizontalCenter: parent.horizontalCenter

                        // 이미지 버튼
                        Image {
                            id: micImage
                            source: "qrc:/icons/mic.png"
                            anchors.fill: parent
                            fillMode: Image.PreserveAspectFit
                        }

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                insightsPanel.micActive = !insightsPanel.micActive
                            }
                        }

                        // 🔴 Pulse 애니메이션 (micActive=true일 때만 보임)
                        Item {
                            width: 20
                            height: 20
                            anchors.centerIn: parent
                            visible: insightsPanel.micActive

                            Rectangle {
                                anchors.fill: parent
                                radius: width / 2
                                color: "#4FC3F7"
                            }

                            Rectangle {
                                anchors.centerIn: parent
                                width: 50
                                height: 50
                                radius: width / 2
                                color: "#4FC3F7"
                                opacity: 0.4

                                SequentialAnimation on scale {
                                    loops: Animation.Infinite
                                    running: insightsPanel.micActive
                                    NumberAnimation { from: 1.0; to: 2.0; duration: 1200 }
                                }
                                SequentialAnimation on opacity {
                                    loops: Animation.Infinite
                                    running: insightsPanel.micActive
                                    NumberAnimation { from: 0.4; to: 0.0; duration: 1200 }
                                }
                            }
                        }
                    }
                    Text { text: "LLM Voice Assistant"; color: "#4FC3F7"; anchors.horizontalCenter: parent.horizontalCenter; font.pixelSize: 16 }
                    // ===============================
                    // 🎤 Processing 텍스트
                    // ===============================
                    Text {
                        text: "Processing command..."
                        color: "#ccc"
                        font.pixelSize: 14
                        anchors.horizontalCenter: parent.horizontalCenter
                        visible: insightsPanel.micActive   // 🎯 클릭하면 나타나고 / 사라짐
                    }

                }
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
            Layout.preferredHeight: parent.height * 0.08
            color: "#101214"     // 배경 (필요하면 변경)
            //border.color: "#0324fc"
            //border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 40
                anchors.rightMargin: 40
                spacing: 20

                // --------------------------
                // 메뉴 아이콘 버튼 1 — Dashboard
                // --------------------------
                Column {
                    width: 70
                    spacing: 0
                    anchors.verticalCenter: parent.verticalCenter

                    Image {
                        source: "qrc:/icons/dashboard.png"
                        width: 40; height: 40
                        fillMode: Image.PreserveAspectFit
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    Text {
                        text: "Dashboard"
                        font.pixelSize: 13
                        color: "grey"
                        horizontalAlignment: Text.AlignHCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                // Reports
                Column {
                    width: 70
                    spacing: 4
                    anchors.verticalCenter: parent.verticalCenter

                    Image {
                        source: "qrc:/icons/report.png"
                        width: 40; height: 40
                        fillMode: Image.PreserveAspectFit
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    Text {
                        text: "Reports"
                        font.pixelSize: 13
                        color: "grey"
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                // Settings
                Column {
                    width: 70
                    spacing: 4
                    anchors.verticalCenter: parent.verticalCenter

                    Image {
                        source: "qrc:/icons/setting.png"
                        width: 40; height: 40
                        fillMode: Image.PreserveAspectFit
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    Text {
                        text: "Settings"
                        font.pixelSize: 13
                        color: "grey"
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                // History
                Column {
                    width: 70
                    spacing: 4
                    anchors.verticalCenter: parent.verticalCenter

                    Image {
                        source: "qrc:/icons/history.png"
                        width: 40; height: 40
                        fillMode: Image.PreserveAspectFit
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    Text {
                        text: "History"
                        font.pixelSize: 13
                        color: "grey"
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                }

                // 🔥 오른쪽 텍스트 (화면 우측 정렬)
                // 🔥 오른쪽 여백 밀기
                Item {
                    Layout.fillWidth: true     // 남는 공간을 차지해서 텍스트를 오른쪽으로 밀기
                    height: parent.height

                    Text {
                        text: "Digital Twin Safety Monitoring System - 8조"
                        font.pixelSize: 20
                        color: "#4FC3F7"

                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }
        }
    }

    // 샘플 Alert 데이터
    ListModel {
        id: alertModel
        ListElement { value: "Fall detected — Worker 003" }
        ListElement { value: "No helmet — Worker 011" }
        ListElement { value: "Restricted area access — Worker 002" }
    }
    ListModel {
        id: violationModel

    }
}

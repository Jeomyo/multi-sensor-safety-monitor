import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 2.15

Rectangle {
    id: rootPanel
    color: "#1E1F22"
    radius: 8
    border.color: "#0324fc"
    border.width: 1

    Column {
        id: insightsPanel
        anchors.fill: parent
        anchors.margins: 16
        spacing: 10

        // ============================================================
        // 상태 변수
        // ============================================================
        property int workerCount: 0
        property bool micActive: false
        property string workState: "미정"

        property var prevWorkerStates: ({})
        property var dangerLineRefs: ({})
        property var reasonLineRefs: ({})

        // PPE 통계
        property real safeRatio: 0.0
        property real noHelmetRatio: 0.0
        property real noVestRatio: 0.0


        // ============================================================
        // PPE 통계 업데이트
        // ============================================================
        function updatePPEStats(workers) {
            let total = workers.length
            if (total === 0) return

            let safeCnt = 0
            let noHelmetCnt = 0
            let noVestCnt = 0

            for (let w of workers) {
                let helmet = w.helmet === true
                let vest   = w.vest === true

                if (helmet && vest) safeCnt++
                if (!helmet) noHelmetCnt++
                if (!vest) noVestCnt++
            }

            safeRatio = safeCnt / total
            noHelmetRatio = noHelmetCnt / total
            noVestRatio = noVestCnt / total
        }


        // ============================================================
        // 제목 영역
        // ============================================================
        Text {
            text: "AI INSIGHTS"
            color: "#4FC3F7"
            font.pixelSize: 20
        }
        Rectangle { height: 1; width: parent.width; color: "#444" }

        Text {
            text: "Latest Alerts Log"
            color: "#fff"
            font.pixelSize: 16
        }

        Rectangle {
            id: alertsBox
            width: parent.width
            height: 160
            radius: 6
            color: "#1E1F22"

            Column {
                id: alertsColumn
                anchors.fill: parent
                anchors.margins: 6
                spacing: 4
            }
        }

        Rectangle { height: 1; width: parent.width; color: "#444" }

        // ============================================================
        // PPE Progress
        // ============================================================
        Text {
            text: "PPE Compliance"
            color: "#fff"
            font.pixelSize: 16
        }

        Repeater {
            model: 3
            Row {
                spacing: 10
                height: 18

                Text {
                    text: ["S", "H", "V"][index]
                    color: "#ccc"
                    font.pixelSize: 14
                }

                Rectangle {
                    id: barRect
                    width: {
                        if (index === 0) return 120 * insightsPanel.safeRatio
                        if (index === 1) return 120 * insightsPanel.noHelmetRatio
                        if (index === 2) return 120 * insightsPanel.noVestRatio
                        return 0
                    }
                    height: 12
                    radius: 3
                    color: {
                        if (index === 0) return "#4CAF50"
                        if (index === 1) return "#FFC107"
                        if (index === 2) return "#F44336"
                    }

                    Behavior on width {
                        NumberAnimation { duration: 300; easing.type: Easing.InOutQuad }
                    }
                    Behavior on color {
                        ColorAnimation { duration: 250 }
                    }
                }

                Text {
                    text: {
                        if (index === 0) return Math.round(insightsPanel.safeRatio * 100) + "%"
                        if (index === 1) return Math.round(insightsPanel.noHelmetRatio * 100) + "%"
                        if (index === 2) return Math.round(insightsPanel.noVestRatio * 100) + "%"
                    }
                    color: "#ccc"
                    font.pixelSize: 13
                }
            }
        }

        Rectangle { height: 1; width: parent.width; color: "#444" }

        // ============================================================
        // MIC 버튼
        // ============================================================
        Item {
            width: 80
            height: 80
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: micImage
                source: "qrc:/icons/mic.png"
                anchors.fill: parent
            }

            MouseArea {
                anchors.fill: parent
                onClicked: insightsPanel.micActive = !insightsPanel.micActive
            }
        }

        Text {
            text: "LLM Voice Assistant"
            color: "#4FC3F7"
            anchors.horizontalCenter: parent.horizontalCenter
            font.pixelSize: 16
        }

        Text {
            text: "Processing command..."
            color: "#ccc"
            visible: insightsPanel.micActive
            anchors.horizontalCenter: parent.horizontalCenter
        }

        // ============================================================
        // AI Insights 업데이트 함수
        // ============================================================
        function updateInsights(workers) {

            for (let w of workers) {

                let id = Number(w.track_id)
                let name = w.name

                let isSafeNow = (w.helmet && w.vest)
                let isDangerNow = !isSafeNow

                let reasonList = []
                if (!w.helmet) reasonList.push("헬멧")
                if (!w.vest)   reasonList.push("조끼")
                let reasonStr = reasonList.length ? reasonList.join(" + ") + " 미착용" : ""

                // 1) 최초 상태 저장
                if (!(id in prevWorkerStates)) {

                    if (isDangerNow) {

                        // 위험 라인 생성
                        dangerLineRefs[id] = Qt.createQmlObject(`
                            import QtQuick 2.15
                            Row {
                                spacing: 6
                                height: 22

                                Item {
                                    width: 12
                                    height: 12
                                    anchors.verticalCenter: parent.verticalCenter

                                    Rectangle {
                                        id: pulseRing
                                        anchors.centerIn: parent
                                        width: 12
                                        height: 12
                                        radius: width / 2
                                        color: "red"
                                        opacity: 0.35

                                        SequentialAnimation on scale {
                                            loops: Animation.Infinite
                                            running: true
                                            NumberAnimation { from: 1.0; to: 2.2; duration: 1000 }
                                        }

                                        SequentialAnimation on opacity {
                                            loops: Animation.Infinite
                                            running: true
                                            NumberAnimation { from: 0.35; to: 0.0; duration: 1000 }
                                        }
                                    }

                                    Rectangle {
                                        anchors.centerIn: parent
                                        width: 12
                                        height: 12
                                        radius: 6
                                        color: "red"
                                    }
                                }

                                Text {
                                    text: "ID: ${id} (${name}) - 위험 감지"
                                    color: "#ccc"
                                    font.pixelSize: 14
                                }
                            }
                        `, alertsColumn)

                        // 원인 라인 생성
                        reasonLineRefs[id] = Qt.createQmlObject(`
                            import QtQuick 2.15
                            Text {
                                text: "    ▶ ${reasonStr}"
                                color: "#FFCC00"
                                font.pixelSize: 13
                            }
                        `, alertsColumn)
                    }

                    prevWorkerStates[id] = { danger: isDangerNow, helmet: w.helmet, vest: w.vest }
                    continue
                }

                // 2) 이전 상태 존재
                let prev = prevWorkerStates[id]

                // A) 안전 → 위험
                if (!prev.danger && isDangerNow) {

                    if (dangerLineRefs[id]) dangerLineRefs[id].destroy()
                    if (reasonLineRefs[id]) reasonLineRefs[id].destroy()

                    dangerLineRefs[id] = Qt.createQmlObject(`
                        import QtQuick 2.15
                        Row {
                            spacing: 6
                            height: 22

                            Item {
                                width: 12
                                height: 12
                                anchors.verticalCenter: parent.verticalCenter

                                Rectangle {
                                    id: pulseRing
                                    anchors.centerIn: parent
                                    width: 12
                                    height: 12
                                    radius: width / 2
                                    color: "red"
                                    opacity: 0.35

                                    SequentialAnimation on scale {
                                        loops: Animation.Infinite
                                        running: true
                                        NumberAnimation { from: 1.0; to: 2.2; duration: 1000 }
                                    }

                                    SequentialAnimation on opacity {
                                        loops: Animation.Infinite
                                        running: true
                                        NumberAnimation { from: 0.35; to: 0.0; duration: 1000 }
                                    }
                                }

                                Rectangle {
                                    anchors.centerIn: parent
                                    width: 12
                                    height: 12
                                    radius: 6
                                    color: "red"
                                }
                            }

                            Text {
                                text: "ID: ${id} (${name}) - 위험 감지"
                                color: "#ccc"
                                font.pixelSize: 14
                            }
                        }
                    `, alertsColumn)

                    reasonLineRefs[id] = Qt.createQmlObject(`
                        import QtQuick 2.15
                        Text {
                            text: "    ▶ ${reasonStr}"
                            color: "#FFCC00"
                            font.pixelSize: 13
                        }
                    `, alertsColumn)
                }

                // B) 위험 유지 중 PPE 이유 변경
                let prevReasonList = []
                if (!prev.helmet) prevReasonList.push("헬멧")
                if (!prev.vest)   prevReasonList.push("조끼")
                let prevReasonStr = prevReasonList.length ? prevReasonList.join(" + ") + " 미착용" : ""

                if (isDangerNow && reasonStr !== prevReasonStr) {
                    if (reasonLineRefs[id]) reasonLineRefs[id].destroy()

                    reasonLineRefs[id] = Qt.createQmlObject(`
                        import QtQuick 2.15
                        Text {
                            text: "    ▶ ${reasonStr}"
                            color: "#FFCC00"
                            font.pixelSize: 13
                        }
                    `, alertsColumn)
                }

                // C) 위험 → 안전
                if (prev.danger && !isDangerNow) {
                    if (dangerLineRefs[id]) { dangerLineRefs[id].destroy(); delete dangerLineRefs[id] }
                    if (reasonLineRefs[id]) { reasonLineRefs[id].destroy(); delete reasonLineRefs[id] }
                }

                // D) 상태 갱신
                prevWorkerStates[id] = { danger: isDangerNow, helmet: w.helmet, vest: w.vest }
            }
        }


        // ============================================================
        // MQTT Signal 처리
        // ============================================================
        Connections {
            target: sensorProvider

            function onWorkersUpdated(workers) {
                insightsPanel.workerCount = workers.length
                insightsPanel.updateInsights(workers)
                insightsPanel.updatePPEStats(workers)
            }

            function onWorkStartModeChanged(v) {
                Qt.createQmlObject(`
                    import QtQuick 2.15
                    Text { text: " ▶ 근무상태: 출근"; color: "#ccc"; font.pixelSize: 13 }
                `, alertsColumn)
            }

            function onWorkEndModeChanged(v) {
                Qt.createQmlObject(`
                    import QtQuick 2.15
                    Text { text: " ▶ 근무상태: 퇴근"; color: "#ccc"; font.pixelSize: 13 }
                `, alertsColumn)
            }
        }
    }
}

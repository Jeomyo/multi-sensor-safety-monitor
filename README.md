import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 2.15

Rectangle {
    id: rootPanel
    color: "#1E1F22"
    radius: 8
    border.color: "#0324fc"
    border.width: 1

    // ============================================================
    // [신규] C++ Backend와 연결 (LLM 응답 수신용)
    // ============================================================
    Connections {
        target: backend
        function onLlmSummaryReady(text) {
            chatDisplay.appendLog("🤖 AI: " + text, "#00CED1"); // 청록색
            micStatusText.text = "대기 중...";
        }
    }

    // ============================================================
    // [기존] 상태 변수 및 로직 (삭제하면 안 됨!)
    // ============================================================
    property int workerCount: 0
    property bool micActive: false
    property string workState: "미정"

    property var prevWorkerStates: ({})
    property var dangerLineRefs: ({})
    property var reasonLineRefs: ({})

    // PPE 통계 변수
    property real safeRatio: 0.0
    property real noHelmetRatio: 0.0
    property real noVestRatio: 0.0

    // [기존] PPE 통계 계산 함수
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

    // [기존] AI Insights 업데이트 함수 (알림 로그 생성)
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
                    createAlertLine(id, name, reasonStr)
                }
                prevWorkerStates[id] = { danger: isDangerNow, helmet: w.helmet, vest: w.vest }
                continue
            }

            // 2) 상태 변화 감지
            let prev = prevWorkerStates[id]

            // 위험 발생 (안전 -> 위험)
            if (!prev.danger && isDangerNow) {
                clearAlertLine(id)
                createAlertLine(id, name, reasonStr)
            }
            // 위험 이유 변경
            else if (isDangerNow && reasonStr !== (prev.helmet ? "" : "헬멧") + (prev.vest ? "" : "조끼")) { // 단순화된 비교
                 // (상세 로직은 생략하되, 업데이트 필요시 여기서 처리)
            }
            // 해제 (위험 -> 안전)
            else if (prev.danger && !isDangerNow) {
                clearAlertLine(id)
            }

            prevWorkerStates[id] = { danger: isDangerNow, helmet: w.helmet, vest: w.vest }
        }
    }

    // 알림 라인 생성 헬퍼 함수
    function createAlertLine(id, name, reasonStr) {
        dangerLineRefs[id] = Qt.createQmlObject(`
            import QtQuick 2.15
            Text { text: "ID: ${id} (${name}) - 위험 감지"; color: "#ccc"; font.pixelSize: 14 }
        `, alertsColumn)

        reasonLineRefs[id] = Qt.createQmlObject(`
            import QtQuick 2.15
            Text { text: "    ▶ ${reasonStr}"; color: "#FFCC00"; font.pixelSize: 13 }
        `, alertsColumn)
    }

    // 알림 라인 삭제 헬퍼 함수
    function clearAlertLine(id) {
        if (dangerLineRefs[id]) { dangerLineRefs[id].destroy(); delete dangerLineRefs[id] }
        if (reasonLineRefs[id]) { reasonLineRefs[id].destroy(); delete reasonLineRefs[id] }
    }


    // ============================================================
    // [기존] MQTT Signal 처리 (센서 데이터 수신)
    // ============================================================
    Connections {
        target: sensorProvider // main.cpp에 등록된 C++ 객체

        function onWorkersUpdated(workers) {
            rootPanel.workerCount = workers.length
            rootPanel.updateInsights(workers)
            rootPanel.updatePPEStats(workers)
        }
    }

    // ============================================================
    // 화면 레이아웃
    // ============================================================
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 10

        // 1. 제목
        Text {
            text: "AI INSIGHTS"
            color: "#4FC3F7"
            font.pixelSize: 20
            Layout.alignment: Qt.AlignLeft
        }
        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // 2. 알림 로그 (기존 기능)
        Text {
            text: "Latest Alerts Log"
            color: "#fff"
            font.pixelSize: 16
        }

        Rectangle {
            id: alertsBox
            Layout.fillWidth: true
            Layout.preferredHeight: 100 // 높이를 줄여서 채팅창 공간 확보
            radius: 6
            color: "#1E1F22"
            border.color: "#333"
            border.width: 1

            ScrollView { // 로그가 많아지면 스크롤 되도록
                anchors.fill: parent
                Column {
                    id: alertsColumn
                    width: parent.width
                    anchors.margins: 6
                    spacing: 4
                }
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // 3. PPE 그래프 (기존 기능)
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
                    width: {
                        if (index === 0) return 120 * rootPanel.safeRatio
                        if (index === 1) return 120 * rootPanel.noHelmetRatio
                        if (index === 2) return 120 * rootPanel.noVestRatio
                        return 0
                    }
                    height: 12
                    radius: 3
                    color: ["#4CAF50", "#FFC107", "#F44336"][index]
                    Behavior on width { NumberAnimation { duration: 300 } }
                }
                Text {
                    text: {
                        let ratio = 0
                        if (index === 0) ratio = rootPanel.safeRatio
                        if (index === 1) ratio = rootPanel.noHelmetRatio
                        if (index === 2) ratio = rootPanel.noVestRatio
                        return Math.round(ratio * 100) + "%"
                    }
                    color: "#ccc"
                    font.pixelSize: 13
                }
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // 4. [신규] AI 비서 (채팅 + 음성)
        Text {
            text: "AI Assistant"
            color: "#4FC3F7"
            font.pixelSize: 16
            Layout.topMargin: 5
        }

        // 채팅창
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true // 남은 공간 다 쓰기
            color: "#121212"
            radius: 5
            clip: true

            ScrollView {
                id: chatScroll
                anchors.fill: parent
                anchors.margins: 10

                TextEdit {
                    id: chatDisplay
                    width: parent.width
                    text: "시스템: AI 비서가 준비되었습니다.\n"
                    color: "#E0E0E0"
                    font.pixelSize: 13
                    wrapMode: TextEdit.Wrap
                    readOnly: true
                    textFormat: TextEdit.RichText

                    function appendLog(msg, colorCode) {
                        var formattedMsg = `<font color="${colorCode}">${msg}</font><br>`;
                        append(formattedMsg);
                        cursorPosition = length - 1;
                    }
                }
            }
        }

        // 입력 컨트롤 (마이크, 텍스트)
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            // 마이크 버튼
            Button {
                id: micBtn
                text: "🎤"
                Layout.preferredWidth: 40
                Layout.preferredHeight: 40
                background: Rectangle {
                    color: micBtn.down ? "#FF4500" : "#4169E1"
                    radius: 5
                }
                contentItem: Text {
                    text: micBtn.text
                    color: "white"
                    font.pixelSize: 18
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    backend.startVoiceCommand();
                    chatDisplay.appendLog("\n🎤 (음성 듣는 중...)", "yellow");
                    micStatusText.text = "음성 인식 중...";
                }
            }

            // 텍스트 입력
            TextField {
                id: inputField
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                placeholderText: "질문 입력 (예: 현재 온도?)"
                color: "white"
                background: Rectangle { color: "#333"; radius: 5 }
                font.pixelSize: 13
                onAccepted: sendBtn.clicked()
            }

            // 전송 버튼
            Button {
                id: sendBtn
                text: "전송"
                Layout.preferredWidth: 50
                Layout.preferredHeight: 40
                background: Rectangle {
                    color: "#2E8B57"
                    radius: 5
                }
                contentItem: Text {
                    text: sendBtn.text
                    color: "white"
                    font.bold: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                onClicked: {
                    if (inputField.text !== "") {
                        backend.sendTextQuery(inputField.text);
                        chatDisplay.appendLog("\n👤 나: " + inputField.text, "white");
                        micStatusText.text = "AI 분석 중...";
                        inputField.text = "";
                    }
                }
            }
        }

        // 상태 메시지
        Text {
            id: micStatusText
            text: "대기 중..."
            color: "gray"
            font.pixelSize: 11
            Layout.alignment: Qt.AlignRight
        }
    }
}

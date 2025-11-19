import QtQuick 6.0
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: rootPage
    // [수정] 배경색을 최상위 Rectangle에서만 정의 (Duplicate error 방지)
    color: "#2C3E50"

    // =======================================================
    // QML -> C++ -> QML 통신 로직 (유지)
    // =======================================================
    Connections {
        target: backend
        function onLlmSummaryReady(summaryText) {
            resultArea.text = "✅ 요약 완료:\n" + summaryText;
            statusLabel.text = "명령 대기 중";
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 50
        spacing: 20

        // 1. 상태 제목
        Text {
            id: statusLabel
            text: "AI 음성 비서 (MQTT 연결됨)"
            font.pointSize: 22
            // [핵심 수정] color 속성 제거 (중복 할당 방지)
            // color: "#FFFFFF"
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 20
        }

        // 2. 명령어 가이드
        Text {
            text: "버튼을 클릭하고 명령을 말씀하세요."
            font.pointSize: 14
            color: "#AAAAAA" // [유지] 이 텍스트는 색상 구분이 필요하므로 유지
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 20
        }

        // 3. 음성 명령 시작 버튼
        Button {
            id: voiceButton
            text: "🎤 음성 명령 시작"
            Layout.preferredWidth: 300
            height: 55
            font.pointSize: 18
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 10

            // 버튼 스타일링 코드 제거 (Duplicate error 방지)
            // background: Rectangle { ... } 코드를 제거합니다.

            onClicked: {
                backend.startVoiceCommand();
                resultArea.text = "🎤 마이크 켜짐: 명령을 말씀하세요...\n\n"
                statusLabel.text = "🟡 음성 인식 중..."
            }
        }

        // 4. 결과 표시 영역 (가장 기본적인 TextEdit만 사용)
        TextEdit {
            id: resultArea

            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.topMargin: 20
            Layout.bottomMargin: 20

            readOnly: true
            text: "LLM 요약 결과가 여기에 표시됩니다. \n\n버튼을 눌러 시스템을 테스트하세요."
            font.pointSize: 14
            // [핵심 수정] color 속성 제거 (중복 할당 방지)
            // color: "#EAEAEA"
            wrapMode: TextEdit.Wrap

            // 배경/내용물 스타일링 코드를 모두 제거 (Invalid property error 방지)
        }
    }
}

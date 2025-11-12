import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: loginPage
    anchors.fill: parent
    color: "#1E1F25"      // 배경색

    Rectangle {
        id: loginCard
        width: parent.width * 0.35
        height: parent.height * 0.5
        color: "#2A303C"
        radius: 12
        anchors.centerIn: parent
        border.color: "#444"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 30
            spacing: 20

            // 제목
            Text {
                text: "Smart Factory 로그인"
                color: "white"
                font.pixelSize: 24
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }

            // 아이디 입력
            TextField {
                id: idField
                placeholderText: "아이디를 입력하세요"
                font.pixelSize: 16
                color: "white"
                background: Rectangle {
                    color: "#3A3F4B"
                    radius: 6
                }
            }

            // 비밀번호 입력
            TextField {
                id: pwField
                placeholderText: "비밀번호를 입력하세요"
                echoMode: TextInput.Password
                font.pixelSize: 16
                color: "white"
                background: Rectangle {
                    color: "#3A3F4B"
                    radius: 6
                }
            }

            // 로그인 버튼
            Button {
                text: "로그인"
                Layout.fillWidth: true
                font.pixelSize: 18
                background: Rectangle {
                    color: "#0078D7"
                    radius: 6
                }

                onClicked: {
                    if (idField.text === "" || pwField.text === "") {
                        console.log("⚠️ 입력값이 비어 있습니다.")
                        return
                    }

                    // ✅ 백엔드 호출
                    backend.login(idField.text, pwField.text)
                }
            }

            // 상태 메시지 표시
            Text {
                id: statusText
                text: ""
                color: "lightgray"
                font.pixelSize: 14
                Layout.alignment: Qt.AlignHCenter
            }

            SequentialAnimation {
                id: fadeAnim
                running: false
                PropertyAnimation { target: statusText; property: "opacity"; from: 0; to: 1; duration: 200 }
                PauseAnimation { duration: 2000 }
                PropertyAnimation { target: statusText; property: "opacity"; from: 1; to: 0.3; duration: 600 }
            }
        }
        // ✅ Backend 시그널 연결
        Connections {
            target: backend

            // 로그인 실패 시
            function onLoginFailed() {
                statusText.text = "로그인 실패"
                statusText.color = "red"
                fadeAnim.start()
            }
        }


    }
}

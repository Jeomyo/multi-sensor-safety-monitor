import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: loginPage
    anchors.fill: parent
    color: "black"

    // ============================
    // 1) 배경 이미지
    // ============================
    Image {
        anchors.fill: parent
        source: "qrc:/icons/bg_login.png"
        fillMode: Image.PreserveAspectCrop
    }

    // ============================
    // 2) ID/PW 패널 이미지 (수동 위치)
    // ============================
    Image {
        id: panelImg
        source: "qrc:/icons/login_panel.png"
        width: 300        // 직접 조절 가능
        height: 200       // 직접 조절 가능

        // 🔥 원하는 위치 직접 조절
        x: 900
        y: 250

        fillMode: Image.PreserveAspectFit
    }

    // ============================
    // 3) 문구 (수동 위치)
    // ============================
    Text {
        id: leftStatusText
        text: "안녕하세요. 안전 관리자님"
        color: "white"
        font.pixelSize: 32
        font.bold: true

        // 🔥 직접 조절
        x: 300
        y: 200
    }

    // ============================
    // 4) 아이디 입력창 (수동 위치)
    // ============================
    TextField {
        id: idField
        width: 300
        height: 40
        placeholderText: "아이디를 입력하세요."
        placeholderTextColor: "white"

        // 🔥 직접 조절 (패널 위에서 정확히 위치)
        x: panelImg.x + 20
        y: panelImg.y + 69

        color: "white"
        background: Rectangle {
            color: "transparent"
            radius: 8
        }
    }

    // ============================
    // 5) 비밀번호 입력창 (수동 위치)
    // ============================
    TextField {
        id: pwField
        width: 300
        height: 40
        placeholderText: "비밀번호를 입력하세요."
        echoMode: TextInput.Password
        placeholderTextColor: "white"

        // 🔥 직접 조절
        x: panelImg.x + 20
        y: panelImg.y + 130

        color: "white"
        background: Rectangle {
            color: "transparent"
            radius: 8
        }
    }

    // ============================
    // 6) 로그인 버튼 (수동 위치)
    // ============================
    Rectangle {
        id: loginButton
        width: 300
        height: 50
        x: panelImg.x
        y: panelImg.y + 180

        radius: 12
        color: "#0A84FF"

        Text {
            anchors.centerIn: parent
            text: "로그인"
            font.pixelSize: 18
            font.bold: true
            color: "white"
        }

        MouseArea {
            anchors.fill: parent
            onClicked: {
                if (idField.text === "" || pwField.text === "") {
                    leftStatusText.color = "orange"
                    leftStatusText.text = "⚠ 아이디와 비밀번호를 입력해주세요."
                    return
                }
                loginManager.login(idField.text, pwField.text)
            }
        }
    }

    // ============================
    // 7) 로그인 결과 처리
    // ============================
    Connections {
        target: loginManager

        function onLoginSucceeded() {
            leftStatusText.color = "#4CAF50"
            leftStatusText.text = "로그인 성공!"
            loginPage.visible = false
            mainDashboard.visible = true
        }

        function onLoginFailed() {
            leftStatusText.color = "red"
            leftStatusText.text = "잘못된 계정입니다"
        }
    }
}

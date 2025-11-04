import QtQuick
import QtQuick.Controls
import logintest 1.0

ApplicationWindow {
    id: root
    width: 1000
    height: 600
    visible: true
    title: "Smart Factory Dashboard"

    // ✅ 메인 화면 StackView
    StackView {
        id: stack
        anchors.fill: parent
        initialItem: "LoginPage.qml"
    }

    // ✅ 백엔드에서 시그널 받을 때 화면 전환
    Connections {
        target: backend
        function onLoginSuccess() {
            console.log("✅ 로그인 성공")
            stack.push("Main.qml")
        }

        function onLoginFailed() {
            console.log("❌ 로그인 실패 → 알림 표시")
            // 나중에 팝업 추가 가능
        }
    }
}

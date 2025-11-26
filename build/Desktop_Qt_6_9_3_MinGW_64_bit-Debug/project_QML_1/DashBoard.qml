import QtQuick
import QtQuick.Controls

Item {
    id: dashboardPage
    anchors.fill: parent

    // 💡 센서 값 실시간 반영
    // ✅ sensorProvider를 QML 전역에서 받을 수 있도록 명시
    Column {
        anchors.left: parent.left
        anchors.right: parent.right
        spacing: 6

        Text {
            text: "ENVIRONMENT SENSORS"
            color: "#4FC3F7"
            font.pixelSize: 20
        }

        Rectangle {
            height: 1
            width: parent.width
            color: "#444"
        }
    }

    Column {
        anchors.top: parent.top
        anchors.topMargin: 40
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 10   // 세로 간격

        Gauge {
            title: "온도"
            value: sensorProvider ? sensorProvider.temperature : 0
            gaugeColor: "#4CAF50"
            label: "Temperature"
            unit: "°C"       // ★ 단위 변경
        }

        Gauge {
            title: "습도"
            value: sensorProvider ? sensorProvider.humidity : 0
            gaugeColor: "#FFC107"
            label: "Humidity"
            unit: "%"        // ★ 단위 변경
        }

        Gauge {
            title: "대기질"
            value: sensorProvider ? sensorProvider.dust : 0
            gaugeColor: "#F44336"
            label: "PM2.5"
            unit: "㎍/㎥"    // ★ 단위 변경
        }

    }

    /*
    // ⚠️ 알림 표시 예시 (필요 시 다시 활성화)
    Row {
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 40
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 10
        visible: true

        Image {
            source: "qrc:/icons/warning.png"
            width: 40; height: 40
        }
        Text {
            text: "A구역 사고 발생!"
            font.pixelSize: 24
            color: "white"
        }
    }
    */
}

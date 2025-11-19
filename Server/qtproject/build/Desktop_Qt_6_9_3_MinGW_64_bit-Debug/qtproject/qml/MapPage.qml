import QtQuick
import QtQuick.Controls


Rectangle {
    id: mapPage
    color: "#202531"

    // 부모(main.qml)에서 전달받을 변수
    property bool workerAWearingHelmet: false
    property bool workerBWearingHelmet: true

    // 제목
    Text {
        text: "공장 내 작업자 위치"
        color: "white"
        font.pixelSize: 22
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 30
    }

    // 🏭 공장 전체 영역
    Rectangle {
        id: factoryArea
        anchors.centerIn: parent
        width: 600
        height: 400
        color: "#2F3640"
        radius: 8
        border.color: "#555"
        border.width: 2
    }

    Image {
        source: "qrc:/images/map.png"
        width: 550
        height: 350
        x: 120
        y: 100
        smooth: true
        visible: true
        fillMode: Image.PreserveAspectFit
    }

    // 작업자 A
    Rectangle {
        width: 20
        height: 20
        radius: 10
        color: "red"
        border.color: "white"
        border.width: 2
        x: 350
        y: 300
        visible: !workerAWearingHelmet

        ToolTip.visible: mared.containsMouse
        ToolTip.text: "작업자 A (안전모 미착용)"
        MouseArea { id: mared; anchors.fill: parent; hoverEnabled: true }
    }
    Text {
        text: "👷‍"
        color: "white"
        font.pixelSize: 22
        x: 350
        y: 300
        visible: workerAWearingHelmet
        anchors.topMargin: 30
        ToolTip.visible: mahelmet.containsMouse
        ToolTip.text: "작업자 A (안전모 착용)"
        MouseArea { id: mahelmet; anchors.fill: parent; hoverEnabled: true }
    }

    // 작업자 B
    Rectangle {
        width: 20
        height: 20
        radius: 10
        color: "red"
        border.color: "white"
        border.width: 2
        x: 450
        y: 300
        visible: !workerBWearingHelmet

        ToolTip.visible: mbred.containsMouse
        ToolTip.text: "작업자 A (안전모 미착용)"
        MouseArea { id: mbred; anchors.fill: parent; hoverEnabled: true }
    }
    Text {
        text: "👷‍"
        color: "white"
        font.pixelSize: 22
        x: 450
        y: 300
        visible: workerBWearingHelmet
        anchors.topMargin: 30
        ToolTip.visible: mbhelmet.containsMouse
        ToolTip.text: "작업자 B (안전모 착용)"
        MouseArea { id: mbhelmet; anchors.fill: parent; hoverEnabled: true }
    }
}

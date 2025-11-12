import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Rectangle {
    id: root
    width: 150
    height: 60
    radius: 4

    // ✅ 외부에서 접근 가능한 속성
    property alias text: label.text
    property bool isSelected: false
    signal clicked()

    // ✅ 내부 상태: 마우스가 올라와 있는지 여부
    property bool hovered: false

    // ✅ 색상 로직을 한 곳에서만 관리 (항상 최신 상태 반영)
    color: {
        if (isSelected) return "#3E4554"
        if (hovered) return "#2A303C"
        return "#202531"
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true

        // ✅ 색상 변경은 위의 color 바인딩에서 처리되므로, 여기서는 상태만 갱신
        onEntered: root.hovered = true
        onExited: root.hovered = false
        onClicked: root.clicked()
    }

    Item {
        id: layoutContainer
        anchors.fill: parent

        Label {
            id: label
            anchors.centerIn: parent
            color: "#E0E0E0"
            text: "대시보드"
        }
    }
}

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import logintest 1.0

Rectangle {
    id: gauge
    width: 200
    height: 200
    radius: width / 2
    color: Theme.colorBgCard
    border.color: "#222"
    border.width: 1
    // anchors.centerIn: parent

    // ===============================
    // 게이지 속성
    // ===============================
    property real value: 72
    property real from: 0
    property real to: 100
    property string unit: "%"
    property color progressColor: getColor(value)
    property string label: "Value"

    // ===============================
    // 배경 원호 (회색)
    // ===============================
    Shape {
        anchors.fill: parent
        ShapePath {
            strokeColor: "#444"
            strokeWidth: 12
            capStyle: ShapePath.RoundCap
            fillColor: "transparent"

            startX: width / 2
            startY: height / 2
            PathAngleArc {
                centerX: width / 2
                centerY: height / 2
                radiusX: width / 2 - 12
                radiusY: height / 2 - 12
                startAngle: -135
                sweepAngle: 270
            }

        }
    }

    // ===============================
    // 진행 원호 (값 표시)
    // ===============================
    Shape {
        anchors.fill: parent
        ShapePath {
            strokeColor: gauge.progressColor
            strokeWidth: 12
            capStyle: ShapePath.RoundCap
            fillColor: "transparent"

            startX: width / 2
            startY: height / 2
            PathAngleArc {
                centerX: width / 2
                centerY: height / 2
                radiusX: width / 2 - 12
                radiusY: height / 2 - 12
                startAngle: -135
                sweepAngle: 270 * (gauge.value - gauge.from) / (gauge.to - gauge.from)
            }

        }
    }

    // ===============================
    // 중앙 텍스트 (값 + 상태)
    // ===============================
    Column {
        anchors.centerIn: parent
        spacing: 4

        Text {
            text: gauge.label
            color: "#aaa"
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            text: Math.round(gauge.value) + " " + gauge.unit
            color: Theme.colorTextLight
            font.pixelSize: 26
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            text: gauge.value < 50 ? "Normal" :
                  gauge.value < 80 ? "Caution" : "Warning"
            color: gauge.progressColor
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
        }
    }

    // ===============================
    // 값 변화 애니메이션
    // ===============================
    Behavior on value {
        NumberAnimation { duration: 600; easing.type: Easing.InOutQuad }
    }

    // ===============================
    // 색상 단계 함수
    // ===============================
    function getColor(val) {
        if (val < 50) return "#4CAF50"   // green
        if (val < 80) return "#FFC107"   // yellow
        return "#F44336"                 // red
    }
}

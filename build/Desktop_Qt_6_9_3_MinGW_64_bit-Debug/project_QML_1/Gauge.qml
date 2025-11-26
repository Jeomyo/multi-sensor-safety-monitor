import QtQuick

Rectangle {
    id: gauge
    width: 160
    height: 160
    color: "transparent"

    property string title: "게이지"
    property real value: 50
    property color gaugeColor: "#4CAF50"
    property string label: "Normal"

    property string unit: "점"   // ✅ 기본 단위 추가

    // 💡 값 변화 시 색상 자동 변경
    onValueChanged: {
        if (value <= 40) {
            gaugeColor = "#4CAF50"
            label = "Normal"
        } else if (value <= 70) {
            gaugeColor = "#FFC107"
            label = "Caution"
        } else {
            gaugeColor = "#F44336"
            label = "Warning"
        }
        canvas.requestPaint()
    }

    Canvas {
        id: canvas
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d");
            ctx.clearRect(0, 0, width, height);

            var centerX = width / 2;
            var centerY = height / 2;
            var radius = 70;
            var startAngle = Math.PI * 0.75;
            var endAngle = Math.PI * 2.25;

            ctx.beginPath();
            ctx.arc(centerX, centerY, radius + 5, 0, 2 * Math.PI);
            ctx.fillStyle = "rgba(255, 255, 255, 0.05)";
            ctx.fill();

            ctx.lineWidth = 10;
            ctx.strokeStyle = "rgba(255,255,255,0.1)";
            ctx.beginPath();
            ctx.arc(centerX, centerY, radius, startAngle, endAngle);
            ctx.stroke();

            ctx.lineWidth = 10;
            ctx.strokeStyle = gaugeColor;
            ctx.beginPath();
            var gaugeEnd = startAngle + (Math.PI * 1.5) * (value / 100.0);
            ctx.arc(centerX, centerY, radius, startAngle, gaugeEnd);
            ctx.stroke();
        }
    }

    Behavior on value {
        NumberAnimation {
            duration: 700
            easing.type: Easing.InOutQuad
        }
    }

    Column {
        anchors.centerIn: parent
        spacing: 5

        Text {
            text: gauge.title
            font.pixelSize: 14
            color: "lightgray"
            horizontalAlignment: Text.AlignHCenter
            width: parent.width
        }

        // ★ 여기서 단위를 교체 ★
        Text {
            text: Math.round(gauge.value) + " " + gauge.unit
            font.pixelSize: 24
            font.bold: true
            color: "white"
            horizontalAlignment: Text.AlignHCenter
            width: parent.width
        }

        Text {
            text: gauge.label
            font.pixelSize: 14
            color: gaugeColor
        }
    }
}

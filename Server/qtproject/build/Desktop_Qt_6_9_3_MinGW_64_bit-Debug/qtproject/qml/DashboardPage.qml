import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: dashboard
    width: parent ? parent.width : 1000
    height: parent ? parent.height : 600
    color: "#1E1F25"   // ✅ 다크 배경 유지

    Connections {
        id: backendConn
        target: backend

        // ✅ 최신 문법: function on<SignalName>(args) { ... }
        function onGauge1ValueChanged(val) {
            const rounded = Math.round(val * 100) / 100;   // 소수 둘째자리
            console.log("📊 게이지 업데이트:", rounded);
            gauge1.value = rounded;
        }
    }

    // 🌡💧 온습도 종합 점수 게이지
    GaugeView {
        id: gauge1
        label: "게이지1"
        value: 0
        from: 0
        to: 100
        unit: "점"

        // ✅ Loader 안에서도 확실히 반영되는 방식
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.leftMargin: 50   // ← 왼쪽 메뉴에서 떨어진 거리
        anchors.topMargin: 20    // ← 위쪽에서 떨어진 거리
    }
    GaugeView {
        label: "게이지2"
        value: 50
        from: 0
        to: 100
        unit: "점"

        // ✅ Loader 안에서도 확실히 반영되는 방식
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.leftMargin: 300   // ← 왼쪽 메뉴에서 떨어진 거리
        anchors.topMargin: 20    // ← 위쪽에서 떨어진 거리
    }
    //미세먼지 농도
    GaugeView {
        label: "미세먼지 농도"
        value: 90
        from: 0
        to: 100
        unit: "㎍/㎥"

        // ✅ Loader 안에서도 확실히 반영되는 방식
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.leftMargin: 550   // ← 왼쪽 메뉴에서 떨어진 거리
        anchors.topMargin: 20    // ← 위쪽에서 떨어진 거리
    }

    Text {
        text: "⚠ A구역 사고 발생!"
        color: "white"
        font.bold: true
        font.pixelSize: 30
        x: 300
        y: 400
        visible: true
    }
}

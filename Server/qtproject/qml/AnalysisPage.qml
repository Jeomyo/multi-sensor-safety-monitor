import QtQuick

Rectangle {
    color: "#202531"

    Column {
        anchors.centerIn: parent
        spacing: 30

        // 상단 제목
        Text {
            text: "데이터 분석 현황"
            color: "white"
            font.pixelSize: 28
            font.bold: true
            anchors.horizontalCenter: parent.horizontalCenter
        }

        // 주요 데이터 카드
        Rectangle {
            width: 450
            height: 300
            radius: 12
            color: "#2b2e35"
            border.color: "#555"
            border.width: 1.5

            Column {
                anchors.centerIn: parent
                spacing: 20

                Text {
                    text: "온도: 26°C"
                    color: "white"
                    font.pixelSize: 20
                    font.bold: true
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "습도: 58%"
                    color: "white"
                    font.pixelSize: 20
                    font.bold: true
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "부상 인원: 1명"
                    color: "#ff6666"
                    font.pixelSize: 20
                    font.bold: true
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "부상자 위치: C 구역"
                    color: "white"
                    font.pixelSize: 18
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "안전모 미착용 인원: 2명"
                    color: "#ff6666"
                    font.pixelSize: 20
                    font.bold: true
                    anchors.horizontalCenter: parent.horizontalCenter
                }

                Text {
                    text: "미착용자 위치: A B 구역"
                    color: "white"
                    font.pixelSize: 18
                    anchors.horizontalCenter: parent.horizontalCenter
                }
            }
        }
    }
}

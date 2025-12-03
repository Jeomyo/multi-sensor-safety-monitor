import QtQuick
import QtQuick.Controls
import QtQuick.Layouts 1.15

Rectangle {
    id: navBar
    anchors.fill: parent
    color: "#101214"

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 40
        anchors.rightMargin: 40
        spacing: 20

        // ----------------------
        // Dashboard
        // ----------------------
        Column {
            width: 70
            spacing: 4
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Image {
                source: "qrc:/icons/dashboard.png"
                width: 40; height: 40
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text { text: "Dashboard"; font.pixelSize: 13; color: "grey" }
        }

        // ----------------------
        // Reports
        // ----------------------
        Column {
            width: 70
            spacing: 4
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Image {
                source: "qrc:/icons/report.png"
                width: 40; height: 40
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text { text: "Reports"; font.pixelSize: 13; color: "grey" }
        }

        // ----------------------
        // Settings
        // ----------------------
        Column {
            width: 70
            spacing: 4
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Image {
                source: "qrc:/icons/setting.png"
                width: 40; height: 40
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text { text: "Settings"; font.pixelSize: 13; color: "grey" }
        }

        // ----------------------
        // History
        // ----------------------
        Column {
            width: 70
            spacing: 4
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            Image {
                source: "qrc:/icons/history.png"
                width: 40; height: 40
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text { text: "History"; font.pixelSize: 13; color: "grey" }
        }

        Item { Layout.fillWidth: true }

        Text {
            text: "Digital Twin Safety Monitoring System - 8조"
            font.pixelSize: 20
            color: "#4FC3F7"
            anchors.verticalCenter: parent.verticalCenter
        }
    }
}

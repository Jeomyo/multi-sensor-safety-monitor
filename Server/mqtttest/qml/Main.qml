import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Rectangle {
    id: mainRoot
    anchors.fill: parent
    color: "#202531"

    property bool workerAWearingHelmet: false
    property bool workerBWearingHelmet: true

    property string selectedMenu: "대시보드 홈"

    RowLayout {
        anchors.fill: parent

        // 🚀 왼쪽 메뉴
        Rectangle {
            Layout.preferredWidth: 200
            Layout.fillHeight: true
            color: "#2A303C"

            NavigationButton {
                text: "대시보드 홈"
                x: 20; y: 120
                isSelected: selectedMenu === text
                onClicked: selectedMenu = text
            }

            NavigationButton {
                text: "지도"
                x: 20; y: 200
                isSelected: selectedMenu === text
                onClicked: selectedMenu = text
            }

            NavigationButton {
                text: "데이터 분석"
                x: 20; y: 280
                isSelected: selectedMenu === text
                onClicked: selectedMenu = text
            }
        }

        // 🚀 오른쪽 메인 화면 (Loader 방식)
        Rectangle {
            id: mainArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#202531"

            Loader {
                id: pageLoader
                anchors.fill: parent

                sourceComponent: {
                    if (selectedMenu === "대시보드 홈") return dashboardPage
                    if (selectedMenu === "지도") return mapPage
                    if (selectedMenu === "데이터 분석") return analysisPage
                    return dashboardPage
                }
            }

            // ✅ 페이지 컴포넌트 정의
            Component {
                id: dashboardPage
                DashboardPage { }  // 내부의 x,y 제대로 작동
            }

            Component {
                id: mapPage
                Rectangle {
                    color: "#202531"

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
                        visible: !workerAWearingHelmet
                        anchors.topMargin: 30
                        ToolTip.visible: mahelmet.containsMouse
                        ToolTip.text: "작업자 B (안전모 착용)"
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
                        x: 400
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
                        x: 400
                        y: 300
                        visible: !workerBWearingHelmet
                        anchors.topMargin: 30
                        ToolTip.visible: mbhelmet.containsMouse
                        ToolTip.text: "작업자 B (안전모 착용)"
                        MouseArea { id: mbhelmet; anchors.fill: parent; hoverEnabled: true }
                    }
                }
            }

            Component {
                id: analysisPage
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
            }
        }
    }
}

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import logintest 1.0

Window {
    width: 1000
    height: 600
    visible: true
    title: qsTr("Smart Factory Dashboard")
    color: Theme.colorBgDark

    property string selectedMenu: "대시보드 홈"

    RowLayout {
        anchors.fill: parent

        // 🚀 왼쪽 메뉴
        Rectangle {
            Layout.preferredWidth: 200
            Layout.fillHeight: true
            color: Theme.colorBgCard

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
            color: Theme.colorBgDark

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
                    color: Theme.colorBgDark

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

                    // 🧱 장애물 1
                    Rectangle {
                        width: 160
                        height: 80
                        color: "white"
                        radius: 4
                        x: factoryArea.x + 120
                        y: factoryArea.y + 250
                    }

                    // 🧱 장애물 2
                    Rectangle {
                        width: 200
                        height: 100
                        color: "white"
                        radius: 4
                        x: factoryArea.x + 400
                        y: factoryArea.y + 120
                    }

                    // 🔴 작업자 A (안전모 미착용)
                    Rectangle {
                        width: 20
                        height: 20
                        radius: 10
                        color: "red"
                        border.color: "white"
                        border.width: 2
                        x: 300
                        y: 300

                        // Tooltip 효과
                        ToolTip.visible: ma.containsMouse
                        ToolTip.text: "작업자 A (안전모 미착용)"
                        MouseArea { id: ma; anchors.fill: parent; hoverEnabled: true }
                    }

                    Text {
                        text: "👷‍"
                        color: "white"
                        font.pixelSize: 22

                        x: 400
                        y: 300
                        anchors.topMargin: 30
                        ToolTip.visible: mb.containsMouse
                        ToolTip.text: "작업자 B (안전모 착용)"
                        MouseArea { id: mb; anchors.fill: parent; hoverEnabled: true }
                    }

                    Image {
                        source: "qrc:/images/helmet.png"   // 리소스 파일 경로
                        width: 32                          // 이미지 크기 조정
                        height: 32
                        x: 420                             // 위치 (작업자 B 근처)
                        y: 295
                        smooth: true
                        visible: true
                        fillMode: Image.PreserveAspectFit
                        ToolTip.visible: mc.containsMouse
                        ToolTip.text: "작업자 B 안전모 착용"
                        MouseArea { id: mc; anchors.fill: parent; hoverEnabled: true }
                    }
                }
            }



            Component {
                id: analysisPage

                Rectangle {
                    color: Theme.colorBgDark

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

import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Rectangle {
    id: mainRoot
    anchors.fill: parent
    color: "#202531"

    property string selectedMenu: "대시보드 홈"

    ColumnLayout {
        anchors.fill: parent

        // =============================
        // 📌 공통 상단 헤더 (여기서 시간 출력)
        // =============================
        Rectangle {
            id: header
            Layout.fillWidth: true
            height: 50
            color: "#1C202A"

            Text {
                id: timeText
                text: backend.currentTime     // ← 여기서 Backend 시간 출력!
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.rightMargin: 20
                color: "#ffffff"
                font.pixelSize: 18
            }

            Text {
                text: "안전 모니터링 대시보드"
                anchors.left: parent.left
                anchors.leftMargin: 20
                anchors.verticalCenter: parent.verticalCenter
                color: "#B0BEC5"
                font.pixelSize: 20
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true

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

                NavigationButton {
                    text: "로그아웃"
                    x: 20; y: 400
                    onClicked: backend.relaunchApp()
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

                // 페이지 컴포넌트 정의
                Component {
                    id: dashboardPage
                    DashboardPage { }
                }

                Component {
                    id: mapPage
                    MapPage { }
                }

                Component {
                    id: analysisPage
                    AnalysisPage { }
                }
            }
        }
    }
}

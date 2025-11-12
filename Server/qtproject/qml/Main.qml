import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Rectangle {
    id: mainRoot
    anchors.fill: parent
    color: "#202531"

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
                MapPage { }
            }

            Component {
                id: analysisPage
                AnalysisPage{ }
            }
        }
    }
}

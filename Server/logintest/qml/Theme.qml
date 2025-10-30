pragma Singleton
import QtQuick

QtObject {

    // ===================================
    // 1. 기본 배경 및 텍스트 색상 (Dark Theme)
    // ===================================
    readonly property color colorBgDark: "#202531"      // 'C'olorbBgDark -> 'c'olorBgDark 로 수정
    readonly property color colorBgCard: "#2A303C"      // 'C'olorbBgCard -> 'c'olorBgCard 로 수정
    readonly property color colorTextLight: "#E0E0E0"   // 'C'olorTextLight -> 'c'olorTextLight 로 수정
    readonly property color colorAccent: "#3E4554"      // 'C'olorAccent -> 'c'olorAccent 로 수정

    // ===================================
    // 2. 상태 표시 색상 (Status Colors)
    // ===================================
    readonly property color colorNormal: "#4CAF50"      // 'C'olorNormal -> 'c'olorNormal 로 수정
    readonly property color colorWarning: "#FFC107"     // 'C'olorWarning -> 'c'olorWarning 로 수정
    readonly property color colorError: "#F44336"       // 'C'olorError -> 'c'olorError 로 수정
    readonly property color colorPrimary: "#83A8F4"     // 'C'olorPrimary -> 'c'olorPrimary 로 수정

    // ===================================
    // 3. 레이아웃 및 여백 상수
    // ===================================
    readonly property real radius: 8                    // 'R'adius -> 'r'adius 로 수정
    readonly property real spacing: 10                  // 'S'pacing -> 's'pacing 로 수정
    readonly property real contentMargin: 20            // 'C'ontentMargin -> 'c'ontentMargin 로 수정
}

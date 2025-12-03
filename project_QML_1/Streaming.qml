import QtQuick 2.15
import QtQuick.Controls 2.15
import QtMultimedia 6.5   // 버전 명시해주는 거 추천

Item {
    id: streamingPage
    anchors.fill: parent

    // 외부에서 넘겨줄 값
    property string sourceUrl: ""
    property bool rtspEnable: false   // 집에서는 false, CCTV 쓸 때 true

    // 🎥 RTSP 영상 출력 영역
    VideoOutput {
        id: videoOutput
        anchors.fill: parent
        anchors.margins: 0
        fillMode: VideoOutput.PreserveAspectFit
    }

    // 🎬 실제 스트림 재생기
    MediaPlayer {
        id: player
        videoOutput: videoOutput
        source: rtspEnable && sourceUrl !== "" ? sourceUrl : ""
        autoPlay: rtspEnable
        loops: MediaPlayer.Infinite

        onErrorOccurred: (err, errorString) => {
            console.log("❌ MediaPlayer Error:", err, errorString)
        }
    }

    // rtspEnable 값이 바뀔 때마다 재생/정지 제어
    onRtspEnableChanged: {
        if (rtspEnable && sourceUrl !== "") {
            console.log("▶️ RTSP 시작:", sourceUrl)
            player.play()
        } else {
            console.log("⏹ RTSP 중지")
            player.stop()
        }
    }

    // 주소가 변경될 때도 자동으로 다시 시작
    onSourceUrlChanged: {
        if (rtspEnable && sourceUrl !== "") {
            console.log("🔄 RTSP URL 변경 → 재생:", sourceUrl)
            player.stop()
            player.play()
        }
    }
}

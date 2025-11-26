import QtQuick
import QtQuick.Controls
import QtMultimedia

Item {
    id: streamingPage
    anchors.fill: parent

    property string sourceUrl: ""
    //color: "#1E1F25"
    /*
    Text {
        text: "RTSP 실시간 스트리밍"
        anchors.bottom: videoContainer.top
        anchors.horizontalCenter: videoContainer.horizontalCenter
        anchors.bottomMargin: 10
        color: "white"
        font.pixelSize: 20
    }
    */

    // 🎥 RTSP 영상 출력
    VideoOutput {
        id: videoOutput
        anchors.fill: parent
        anchors.margins: 0
        fillMode: VideoOutput.PreserveAspectFit
    }

    // 🎬 RTSP 스트림 플레이어
    MediaPlayer {
        id: player
        videoOutput: videoOutput
        // ⚠️ VLC 송출 주소 정확히 일치시켜야 합니다.
        //source: "rtsp://127.0.0.1:8554/stream"
        //source: "rtsp://admin:qw12qw12%21@192.168.0.64:554/Streaming/Channels/101"
        //source: "rtsp://admin:qw12qw12%21@192.168.0.64:554/Streaming/Channels/102"
        source: sourceUrl
        autoPlay: true
        loops: MediaPlayer.Infinite

        onErrorOccurred: (err, errorString) => {
            console.log("❌ MediaPlayer Error:", err, errorString)
        }
    }

    // ▶️ 제어 버튼
    /*
    Row {
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottomMargin: 10
        spacing: 10

        Button {
            text: player.playbackState === MediaPlayer.PlayingState ? "일시정지" : "재생"
            onClicked: {
                if (player.playbackState === MediaPlayer.PlayingState)
                    player.pause()
                else
                    player.play()
            }
        }

        Button {
            text: "정지"
            onClicked: player.stop()
        }
    }
    */
}

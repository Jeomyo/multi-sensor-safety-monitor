import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: mapPage
    anchors.fill: parent

    // ============================================================
    // 선택된 작업자 상태 저장
    // ============================================================
    property int selectedWorkerId: -1
    property var selectedWorkerData: null

    // ============================================================
    // Trajectory / Marker 저장용
    // ============================================================
    property var trajectories: ({})
    property var workerMarkers: ({})   // trackId → markerObject

    // ============================================================
    // 이미지 크기
    // ============================================================
    property int imgW: 1325
    property int imgH: 540

    // ============================================================
    // 카메라 기준 위치
    // ============================================================
    //property real targetX: -4.56
    property real targetX: -0.64
    //property real targetY: 0.18
    property real targetY: -1.5
    // ------------------------------------------------------------
    // 가장 가까운 작업자 찾기
    // ------------------------------------------------------------
    function getNearestWorkerName() {
        if (workerListModel.count === 0)
            return "없음"

        let bestName = "없음"
        let bestDist = 99999999

        for (let i = 0; i < workerListModel.count; i++) {
            let w = workerListModel.get(i)
            let dx = w.x - targetX
            let dy = w.y - targetY
            let d = Math.sqrt(dx*dx + dy*dy)

            if (d < bestDist) {
                bestDist = d
                bestName = w.name
            }
        }
        return bestName
    }

    // ------------------------------------------------------------
    // 출근 / 퇴근 / 호출 모드 표시 함수
    // ------------------------------------------------------------
    function showMode(workType) {
        fadeOut.stop()
        modeIndicator.visible = true
        modeIndicator.opacity = 1.0

        let nearest = getNearestWorkerName()
        modeText.text = nearest + " : " + workType

        hideModeTimer.restart()
    }

    // ============================================================
    // 평면도 이미지
    // ============================================================
    Image {
        id: floorImage
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
        source: "qrc:/icons/comp2.jpg"
    }

    // ============================================================
    // 현재 마우스 world 좌표 출력
    // ============================================================
    Text {
        id: coordLabel
        text: ""
        color: "white"
        font.pixelSize: 14
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 10
        anchors.bottomMargin: 10
    }

    // ============================================================
    // 마우스 → world 좌표 변환
    // ============================================================
    MouseArea {
        id: mouseTracker
        anchors.fill: parent
        hoverEnabled: true

        onPositionChanged: (mouse) => {
            let sx = mouse.x
            let sy = mouse.y

            let scale = Math.min(mapPage.width / mapPage.imgW,
                                 mapPage.height / mapPage.imgH)
            let renderW = mapPage.imgW * scale
            let renderH = mapPage.imgH * scale
            let offsetX = (mapPage.width - renderW) / 2
            let offsetY = (mapPage.height - renderH) / 2

            let imgX = (sx - offsetX) / scale
            let imgY = (sy - offsetY) / scale

            if (imgX < 0 || imgX > mapPage.imgW ||
                imgY < 0 || imgY > mapPage.imgH) {
                coordLabel.text = ""
                return
            }

            let det = mapPage.a * mapPage.e - mapPage.b * mapPage.d
            if (det === 0) return

            let ia =  mapPage.e / det
            let ib = -mapPage.b / det
            let ic = (mapPage.b * mapPage.f - mapPage.c * mapPage.e) / det
            let id_ = -mapPage.d / det
            let ie =  mapPage.a / det
            let if_ = (mapPage.c * mapPage.d - mapPage.a * mapPage.f) / det

            let wx = ia * imgX + ib * imgY + ic
            let wy = id_ * imgX + ie * imgY + if_

            coordLabel.text = `X: ${wx.toFixed(2)} , Y: ${wy.toFixed(2)}`
        }

        onExited: coordLabel.text = ""
    }

    // ============================================================
    // 기준점 (world → image)
    // ============================================================
    property var refPoints: [
        { world: Qt.point(2.33, -0.9),      img: Qt.point(31, 29) },
        { world: Qt.point(-5.357, -2.97),   img: Qt.point(1295, 29) },
        { world: Qt.point(1.59, 1.785),     img: Qt.point(31, 512) },
        { world: Qt.point(-6.168, -0.2637), img: Qt.point(1295, 512) }
    ]

    property real a: 1
    property real b: 0
    property real c: 0
    property real d: 0
    property real e: 1
    property real f: 0

    // ------------------------------------------------------------
    // 아핀 변환 계산
    // ------------------------------------------------------------
    function computeAffine() {
        let p = refPoints
        let x1=p[0].world.x, y1=p[0].world.y
        let x2=p[1].world.x, y2=p[1].world.y
        let x3=p[2].world.x, y3=p[2].world.y

        let u1=p[0].img.x, v1=p[0].img.y
        let u2=p[1].img.x, v2=p[1].img.y
        let u3=p[2].img.x, v3=p[2].img.y

        let det = x1*(y2-y3) - y1*(x2-x3) + (x2*y3 - y2*x3)
        if (det === 0) return

        a = (u1*(y2-y3) + u2*(y3-y1) + u3*(y1-y2)) / det
        b = (u1*(x3-x2) + u2*(x1-x3) + u3*(x2-x1)) / det
        c = (u1*(x2*y3 - x3*y2) + u2*(x3*y1-x1*y3) + u3*(x1*y2-x2*y1)) / det

        d = (v1*(y2-y3) + v2*(y3-y1) + v3*(y1-y2)) / det
        e = (v1*(x3-x2) + v2*(x1-x3) + v3*(x2-x1)) / det
        f = (v1*(x2*y3 - x3*y2) + v2*(x3*y1-x1*y3) + v3*(x1*y2-x2*y1)) / det
    }

    // ------------------------------------------------------------
    // world → image
    // ------------------------------------------------------------
    function worldToImage(x, y) {
        return Qt.point(
            a * x + b * y + c,
            d * x + e * y + f
        )
    }

    // ------------------------------------------------------------
    // world → 화면 좌표
    // ------------------------------------------------------------
    function worldToScreen(x, y) {
        let p = worldToImage(x, y)
        let imgX = p.x
        let imgY = p.y

        let scale = Math.min(width / imgW, height / imgH)
        let renderW = imgW * scale
        let renderH = imgH * scale

        let offsetX = (width - renderW) / 2
        let offsetY = (height - renderH) / 2

        return Qt.point(
            offsetX + imgX * scale,
            offsetY + imgY * scale
        )
    }

    // ============================================================
    // 선택된 작업자 정보 업데이트
    // ============================================================
    function updateSelectedFromModel() {
        if (selectedWorkerId === -1) {
            selectedWorkerData = null
            return
        }

        for (let i = 0; i < workerListModel.count; i++) {
            let w = workerListModel.get(i)
            if (w.trackId === selectedWorkerId) {
                selectedWorkerData = {
                    name: w.name,
                    trackId: w.trackId,
                    x: w.x,
                    y: w.y,
                    helmet: w.helmet,
                    vest: w.vest,
                    safe: w.safe,
                    stamp: w.stamp
                }
                return
            }
        }
        selectedWorkerData = null
    }

    // ============================================================
    // Trajectory 그리기
    // ============================================================
    function drawTrajectory() {
        for (let i = trajectoryLayer.children.length - 1; i >= 0; i--)
            trajectoryLayer.children[i].destroy()

        if (selectedWorkerId === -1)
            return

        let tid = selectedWorkerId
        let traj = trajectories[tid]
        if (!traj || traj.length === 0)
            return

        let n = traj.length
        for (let j = 0; j < n; j++) {
            let pt = traj[j]
            let pos = worldToScreen(pt.x, pt.y)
            let alpha = (j + 1) / n

            let dot = `
                import QtQuick 2.15
                Rectangle {
                    width: 8; height: 8
                    radius: 8
                    x: ${pos.x - 2}
                    y: ${pos.y - 2}
                    color: "#4FC3F7"
                    opacity: ${alpha}
                }
            `
            Qt.createQmlObject(dot, trajectoryLayer)
        }
    }

    Component.onCompleted: computeAffine()

    // ============================================================
    // 레이어들
    // ============================================================
    Item {
        id: trajectoryLayer
        anchors.fill: parent
        z: 40
    }

    Item {
        id: workerLayer
        anchors.fill: parent
        z: 50
    }

    ListModel {
        id: workerListModel
    }

    // ============================================================
    // MQTT 업데이트 처리
    // ============================================================
    Connections {
        target: sensorProvider

        function onWorkersUpdated(workers) {
            let activeIds = {}

            // 1) 이번 프레임에 존재하는 ID 수집
            for (let i = 0; i < workers.length; i++)
                activeIds[workers[i].track_id] = true

            // 2) 기존 마커 중에서 이번 프레임에 없는 ID는 삭제
            for (let id in mapPage.workerMarkers) {
                if (!activeIds[id]) {
                    mapPage.workerMarkers[id].destroy()
                    delete mapPage.workerMarkers[id]
                }
            }
            workerListModel.clear()

            for (let i = 0; i < workers.length; i++) {
                let w = workers[i]

                workerListModel.append({
                    "name": w.name,
                    "trackId": w.track_id,
                    "x": w.x,
                    "y": w.y,
                    "helmet": w.helmet,
                    "vest": w.vest,
                    "safe": w.safe,
                    "stamp": w.stamp
                })

                if (!mapPage.trajectories[w.track_id])
                    mapPage.trajectories[w.track_id] = []

                mapPage.trajectories[w.track_id].push({ x: w.x, y: w.y })
                if (mapPage.trajectories[w.track_id].length > 1000)
                    mapPage.trajectories[w.track_id].shift()

                let pos = mapPage.worldToScreen(w.x, w.y)
                let marker = mapPage.workerMarkers[w.track_id]

                if (!marker) {
                    let markerQml = `
                        import QtQuick 2.15
                        Item {
                            id: markerRoot
                            objectName: "workerMarker"
                            property int trackId: ${w.track_id}
                            property bool safe: false
                            width: 40; height: 40

                            property bool enableAnimation: false

                            Behavior on x {
                                enabled: enableAnimation
                                NumberAnimation { duration: 160; easing.type: Easing.InOutQuad }
                            }
                            Behavior on y {
                                enabled: enableAnimation
                                NumberAnimation { duration: 160; easing.type: Easing.InOutQuad }
                            }


                            Rectangle {
                                id: dangerPulse
                                z: 1
                                width: 40
                                height: 40
                                radius: width / 2
                                color: "#ff4444"
                                opacity: 0.4
                                visible: !safe

                                SequentialAnimation on scale {
                                    loops: Animation.Infinite
                                    running: !safe
                                    NumberAnimation { from: 0.6; to: 1.8; duration: 1200 }
                                    NumberAnimation { from: 1.8; to: 0.6; duration: 0 }
                                }

                                SequentialAnimation on opacity {
                                    loops: Animation.Infinite
                                    running: !safe
                                    NumberAnimation { from: 0.5; to: 0.0; duration: 1200 }
                                    NumberAnimation { from: 0.0; to: 0.5; duration: 0 }
                                }
                            }

                            Image {
                                anchors.fill: parent
                                z: 99
                                source: safe ? "qrc:/icons/safe.png" : "qrc:/icons/red_500.png"
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }

                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    if (mapPage.selectedWorkerId === trackId) {
                                        mapPage.selectedWorkerId = -1
                                        mapPage.selectedWorkerData = null
                                    } else {
                                        mapPage.selectedWorkerId = trackId
                                        mapPage.updateSelectedFromModel()
                                    }
                                }
                            }
                        }
                    `
                    marker = Qt.createQmlObject(markerQml, workerLayer)
                    mapPage.workerMarkers[w.track_id] = marker
                }
                marker.enableAnimation = false
                marker.x = pos.x - marker.width / 2
                marker.y = pos.y - marker.height / 2
                marker.safe = w.safe
                Qt.callLater(() => { marker.enableAnimation = true })
            }

            if (mapPage.selectedWorkerId !== -1)
                mapPage.updateSelectedFromModel()

            mapPage.drawTrajectory()
        }
    }

    // ============================================================
    // 선택 변경 시 trajectory 갱신
    // ============================================================
    Connections {
        target: mapPage
        function onSelectedWorkerIdChanged() {
            mapPage.updateSelectedFromModel()
            mapPage.drawTrajectory()
        }
    }

    // ============================================================
    // 선택된 작업자 팝업
    // ============================================================
    Rectangle {
        id: popup
        visible: mapPage.selectedWorkerData !== null
        radius: 6
        width: 100
        height: 100
        z: 200

        color: "#333333CC"
        border.color: "black"
        border.width: 1

        Rectangle {
            anchors.fill: parent
            anchors.margins: 4
            color: "#00000055"
            radius: 4
        }

        function updatePosition() {
            if (!mapPage.selectedWorkerData) return

            let p = mapPage.worldToScreen(mapPage.selectedWorkerData.x,
                                         mapPage.selectedWorkerData.y)

            x = p.x + 15
            y = p.y - height - 5

            if (x + width > mapPage.width) x = mapPage.width - width - 5
            if (x < 0) x = 5
            if (y < 0) y = p.y + 10
            if (y + height > mapPage.height)
                y = mapPage.height - height - 5
        }

        Column {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 2

            Text {
                text: mapPage.selectedWorkerData
                      ? "이름: " + mapPage.selectedWorkerData.name : ""
                color: "yellow"
                font.pixelSize: 11
                font.bold: true
            }
            Text {
                text: mapPage.selectedWorkerData
                      ? "ID: " + mapPage.selectedWorkerData.trackId : ""
                color: "white"
                font.pixelSize: 11
            }
            Text {
                text: mapPage.selectedWorkerData
                      ? "헬멧: " + (mapPage.selectedWorkerData.helmet ? "착용" : "미착용") : ""
                color: "white"
                font.pixelSize: 11
            }
            Text {
                text: mapPage.selectedWorkerData
                      ? "조끼: " + (mapPage.selectedWorkerData.vest ? "착용" : "미착용") : ""
                color: "white"
                font.pixelSize: 11
            }
            Text {
                text: mapPage.selectedWorkerData
                      ? "상태: " + (mapPage.selectedWorkerData.safe ? "안전" : "위험") : ""
                color: mapPage.selectedWorkerData && mapPage.selectedWorkerData.safe
                       ? "#00FF00" : "red"
                font.pixelSize: 11
                font.bold: true
            }
        }

        Connections {
            target: mapPage
            function onSelectedWorkerDataChanged() {
                popup.updatePosition()
            }
        }
    }

    // ============================================================
    // 출근 / 퇴근 / 호출 상태 표시 UI
    // ============================================================
    Rectangle {
        id: modeIndicator
        width: 120
        height: 20
        radius: 8
        anchors.top: parent.top
        anchors.topMargin: 80
        anchors.right: parent.right
        anchors.rightMargin: 30
        z: 999
        visible: false
        opacity: 1.0

        color: "#1E1F22"
        border.color: "white"
        border.width: 0

        Timer {
            id: hideModeTimer
            interval: 3000
            repeat: false
            onTriggered: fadeOut.start()
        }

        NumberAnimation {
            id: fadeOut
            target: modeIndicator
            property: "opacity"
            from: 1.0
            to: 0.0
            duration: 600
            easing.type: Easing.InOutQuad

            onFinished: {
                modeIndicator.visible = false
                modeIndicator.opacity = 1.0
            }
        }

        Row {
            anchors.centerIn: parent
            spacing: 8

            Item {
                width: 20
                height: 20

                Rectangle {
                    anchors.centerIn: parent
                    width: 14
                    height: 14
                    radius: width / 2
                    color: "#4FC3F7"
                    opacity: 0.35

                    SequentialAnimation on scale {
                        loops: Animation.Infinite
                        running: true
                        NumberAnimation { from: 1.0; to: 2.2; duration: 1000 }
                    }

                    SequentialAnimation on opacity {
                        loops: Animation.Infinite
                        running: true
                        NumberAnimation { from: 0.35; to: 0.0; duration: 1000 }
                    }
                }

                Rectangle {
                    anchors.centerIn: parent
                    width: 12
                    height: 12
                    radius: 6
                    color: "#4FC3F7"
                }
            }

            Text {
                id: modeText
                color: "#CCC"
                font.pixelSize: 14
                font.bold: true
            }
        }
    }

    // ============================================================
    // MQTT: 출근 / 퇴근 / 호출 이벤트
    // ============================================================
    Connections {
        target: sensorProvider

        function onWorkStartModeChanged(value) {
            if (value === 1) showMode("출근")
        }
        function onWorkEndModeChanged(value) {
            if (value === 1) showMode("퇴근")
        }
        function onsystemMsgOneReceived(msg) {
            if (msg === "call")         showMode("관리자 호출")
            else if (msg === "problem") showMode("문제 발생")
        }
    }
}

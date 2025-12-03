import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: mapPage
    anchors.fill: parent

    /* =============================
       선택된 작업자 상태
       ============================= */
    property int selectedWorkerId: -1
    property var selectedWorkerData: null

    /* =============================
       Trajectory / Marker 데이터
       ============================= */
    property var trajectories: ({})
    property var workerMarkers: ({})     // trackId → Marker 객체

    /* =============================
       원본 지도 이미지 크기
       ============================= */
    property int imgW: 1325
    property int imgH: 540

    /* =============================
       지도 이미지
       ============================= */
    Image {
        id: floorImage
        anchors.fill: parent
        fillMode: Image.PreserveAspectFit
        source: "qrc:/icons/comp2.jpg"
    }

    /* =============================
       마우스 좌표 (world 좌표 출력)
       ============================= */
    Text {
        id: coordLabel
        text: ""
        color: "white"
        font.pixelSize: 14
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 10
    }

    MouseArea {
        id: mouseTracker
        anchors.fill: parent
        hoverEnabled: true

        onPositionChanged: (mouse)=> {
            let sx = mouse.x
            let sy = mouse.y

            let scale = Math.min(mapPage.width / imgW,
                                 mapPage.height / imgH)
            let renderW = imgW * scale
            let renderH = imgH * scale
            let offsetX = (mapPage.width - renderW) / 2
            let offsetY = (mapPage.height - renderH) / 2

            let imgX = (sx - offsetX) / scale
            let imgY = (sy - offsetY) / scale

            if (imgX < 0 || imgX > imgW || imgY < 0 || imgY > imgH) {
                coordLabel.text = ""
                return
            }

            let det = a * e - b * d
            if (det === 0) return

            let ia =  e / det
            let ib = -b / det
            let ic = (b * f - c * e) / det
            let id_ = -d / det
            let ie =  a / det
            let if_ = (c * d - a * f) / det

            let wx = ia * imgX + ib * imgY + ic
            let wy = id_ * imgX + ie * imgY + if_

            coordLabel.text = `X: ${wx.toFixed(2)}, Y: ${wy.toFixed(2)}`
        }

        onExited: coordLabel.text = ""
    }

    /* =============================
       Affine 변환 (world → image)
       ============================= */
    property var refPoints: [
        { world: Qt.point(2.33, -0.9),   img: Qt.point(31, 29) },
        { world: Qt.point(-5.357, -2.97), img: Qt.point(1295, 29) },
        { world: Qt.point(1.59, 1.785),   img: Qt.point(31, 512) },
        { world: Qt.point(-6.168, -0.2637), img: Qt.point(1295, 512) }
    ]

    property real a: 1
    property real b: 0
    property real c: 0
    property real d: 0
    property real e: 1
    property real f: 0

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

    function worldToImage(x, y) {
        return Qt.point(a * x + b * y + c,
                        d * x + e * y + f)
    }

    function worldToScreen(x, y) {
        let p = worldToImage(x, y)
        let scale = Math.min(width / imgW, height / imgH)

        let renderW = imgW * scale
        let renderH = imgH * scale
        let offsetX = (width - renderW) / 2
        let offsetY = (height - renderH) / 2

        return Qt.point(
            offsetX + p.x * scale,
            offsetY + p.y * scale
        )
    }

    /* =============================
       선택된 작업자 정보 갱신
       ============================= */
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

    /* =============================
       Trajectory 그리기 (dot 방식)
       ============================= */
    function drawTrajectory() {
        for (let i = trajectoryLayer.children.length - 1; i >= 0; i--)
            trajectoryLayer.children[i].destroy()

        if (selectedWorkerId === -1) return

        let traj = trajectories[selectedWorkerId]
        if (!traj || traj.length === 0) return

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

    /* =============================
       레이어들
       ============================= */
    Item { id: trajectoryLayer; anchors.fill: parent; z: 40 }
    Item { id: workerLayer;     anchors.fill: parent; z: 50 }

    ListModel { id: workerListModel }

    /* =============================
       MQTT 업데이트 처리
       ============================= */
    Connections {
        target: sensorProvider

        function onWorkersUpdated(workers) {
            workerListModel.clear()

            for (let i = 0; i < workers.length; i++) {
                let w = workers[i]

                workerListModel.append({
                    name: w.name,
                    trackId: w.track_id,
                    x: w.x, y: w.y,
                    helmet: w.helmet,
                    vest: w.vest,
                    safe: w.safe,
                    stamp: w.stamp
                })

                if (!trajectories[w.track_id])
                    trajectories[w.track_id] = []

                trajectories[w.track_id].push({ x: w.x, y: w.y })
                if (trajectories[w.track_id].length > 1000)
                    trajectories[w.track_id].shift()

                let pos = worldToScreen(w.x, w.y)
                let marker = workerMarkers[w.track_id]

                if (!marker) {
                    let markerQml = `
                        import QtQuick 2.15
                        Item {
                            id: markerRoot
                            property int trackId: ${w.track_id}
                            property bool safe: false
                            width: 40; height: 40

                            Behavior on x { NumberAnimation { duration: 160; easing.type: Easing.InOutQuad } }
                            Behavior on y { NumberAnimation { duration: 160; easing.type: Easing.InOutQuad } }

                            Rectangle {
                                id: pulse
                                anchors.centerIn: parent
                                width: 40; height: 40
                                radius: 20
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
                                source: safe ? "qrc:/icons/safe.png" : "qrc:/icons/red_500.png"
                                fillMode: Image.PreserveAspectFit
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
                    workerMarkers[w.track_id] = marker
                }

                marker.x = pos.x - 10
                marker.y = pos.y - 10
                marker.safe = w.safe
            }

            if (selectedWorkerId !== -1)
                updateSelectedFromModel()

            drawTrajectory()
        }
    }

    /* =============================
       선택 변경 → Trajectory 업데이트
       ============================= */
    Connections {
        target: mapPage
        function onSelectedWorkerIdChanged() {
            updateSelectedFromModel()
            drawTrajectory()
        }
    }

    /* =============================
       팝업 UI
       ============================= */
    Rectangle {
        id: popup
        visible: selectedWorkerData !== null
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
            if (!selectedWorkerData) return

            let p = worldToScreen(selectedWorkerData.x, selectedWorkerData.y)

            x = p.x + 15
            y = p.y - height - 5

            if (x + width > mapPage.width)  x = mapPage.width - width - 5
            if (x < 0)                      x = 5
            if (y < 0)                      y = p.y + 10
            if (y + height > mapPage.height)
                y = mapPage.height - height - 5
        }

        Column {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 2

            Text { text: selectedWorkerData ? "이름: " + selectedWorkerData.name : ""; color: "yellow"; font.pixelSize: 11 }
            Text { text: selectedWorkerData ? "ID: " + selectedWorkerData.trackId : ""; color: "white"; font.pixelSize: 11 }
            Text { text: selectedWorkerData ? "헬멧: " + (selectedWorkerData.helmet ? "착용" : "미착용") : ""; color: "white"; font.pixelSize: 11 }
            Text { text: selectedWorkerData ? "조끼: " + (selectedWorkerData.vest ? "착용" : "미착용") : ""; color: "white"; font.pixelSize: 11 }
            Text { text: selectedWorkerData ? "상태: " + (selectedWorkerData.safe ? "안전" : "위험") : "";
                   color: selectedWorkerData && selectedWorkerData.safe ? "#00FF00" : "red";
                   font.pixelSize: 11; font.bold: true }
        }

        Connections {
            target: mapPage
            function onSelectedWorkerDataChanged() { popup.updatePosition() }
        }
    }
}

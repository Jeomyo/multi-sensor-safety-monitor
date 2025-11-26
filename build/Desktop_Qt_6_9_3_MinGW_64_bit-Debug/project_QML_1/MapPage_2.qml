import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: mapPage
    anchors.fill: parent

    // =============================
    // 맵 제목
    // =============================
    /*
    Text {
        text: "공장 내 작업자 위치"
        anchors.bottom: mapArea.top
        anchors.horizontalCenter: mapArea.horizontalCenter
        anchors.bottomMargin: 10
        color: "white"
        font.pixelSize: 20
    }
    */

        //------------------------------------
        // 선택된 작업자 상태 저장
        //------------------------------------
        property int  selectedWorkerId: -1      // track_id 기준
        property var  selectedWorkerData: null  // {name, trackId, x,y,helmet,vest,safe,stamp}

        //------------------------------------
        // Trajectory / Heatmap 데이터
        //------------------------------------
        // track_id → [{x, y}, ...]
        property var trajectories: ({})
        // heatmap cell 크기 (m 단위)
        property real cellSize: 0.3
        // "cx,cy" → count
        property var heatmap: ({})

        //------------------------------------
        // 원본 이미지 크기
        //------------------------------------
        property int imgW: 1694
        property int imgH: 609

        //------------------------------------
        // 평면도 이미지
        //------------------------------------
        Image {
            id: floorImage
            anchors.fill: parent
            fillMode: Image.PreserveAspectFit
            source: "file:///C:/QT_QML/project_QML_1/comp2.jpg"
        }

        //------------------------------------
        // 커서 world 좌표 표시
        //------------------------------------
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

        MouseArea {
            id: mouseTracker
            anchors.fill: parent
            hoverEnabled: true

            onPositionChanged: (mouse)=> {
                let sx = mouse.x
                let sy = mouse.y

                let scale = Math.min(mapArea.width / mapArea.imgW,
                                     mapArea.height / mapArea.imgH)
                let renderW = mapArea.imgW * scale
                let renderH = mapArea.imgH * scale
                let offsetX = (mapArea.width  - renderW) / 2
                let offsetY = (mapArea.height - renderH) / 2

                let imgX = (sx - offsetX) / scale
                let imgY = (sy - offsetY) / scale

                if (imgX < 0 || imgX > mapArea.imgW || imgY < 0 || imgY > mapArea.imgH) {
                    coordLabel.text = ""
                    return
                }

                let det = mapArea.a * mapArea.e - mapArea.b * mapArea.d
                if (det === 0) return

                let ia =  mapArea.e / det
                let ib = -mapArea.b / det
                let ic = (mapArea.b * mapArea.f - mapArea.c * mapArea.e) / det
                let id_ = -mapArea.d / det
                let ie =  mapArea.a / det
                let if_ = (mapArea.c * mapArea.d - mapArea.a * mapArea.f) / det

                let wx = ia * imgX + ib * imgY + ic
                let wy = id_ * imgX + ie * imgY + if_

                coordLabel.text = `X: ${wx.toFixed(2)} , Y: ${wy.toFixed(2)}`
            }

            onExited: coordLabel.text = ""
        }

        //------------------------------------
        // 기준점 (world → 픽셀)
        //------------------------------------
        property var refPoints: [
            { world: Qt.point(5.46, 3.50), img: Qt.point(20, 20) },
            { world: Qt.point(2.62, 1.37), img: Qt.point(1660, 20) },
            { world: Qt.point(-6.52, 0.74), img: Qt.point(20, 590) },
            { world: Qt.point(1.78, -1.34), img: Qt.point(1660, 590) }
        ]

        //------------------------------------
        // Affine 변환 계수
        //------------------------------------
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
            if (det === 0) {
                console.error("❌ Affine 계산 실패")
                return
            }

            a = (u1*(y2-y3) + u2*(y3-y1) + u3*(y1-y2)) / det
            b = (u1*(x3-x2) + u2*(x1-x3) + u3*(x2-x1)) / det
            c = (u1*(x2*y3 - x3*y2) + u2*(x3*y1-x1*y3) + u3*(x1*y2-x2*y1)) / det

            d = (v1*(y2-y3) + v2*(y3-y1) + v3*(y1-y2)) / det
            e = (v1*(x3-x2) + v2*(x1-x3) + v3*(x2-x1)) / det
            f = (v1*(x2*y3 - x3*y2) + v2*(x3*y1-x1*y3) + v3*(x1*y2-x2*y1)) / det

            console.log("✅ Affine 계산 완료")
        }

        function worldToImage(x, y) {
            return Qt.point(
                a * x + b * y + c,
                d * x + e * y + f
            )
        }

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

        // 선택된 Worker 정보를 workerListModel에서 찾아서 갱신
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
            // 못 찾으면 해제
            selectedWorkerData = null
        }

        //------------------------------------
        // Heatmap 그리기
        //------------------------------------
        function drawHeatmap() {
            // 기존 heatmapLayer 정리
            for (let i = heatmapLayer.children.length - 1; i >= 0; i--)
                heatmapLayer.children[i].destroy()

            let size = cellSize   // world meters per cell

            // 색상 스케일 (원하는 대로 조정 가능)
            function heatColor(count) {
                if (count === 0) return "#FFFFFF00"; // 투명
                else if (count < 3) return "#FFF4C266";  // 밝은 노랑
                else if (count < 8) return "#FFCC6666";  // 주황
                else return "#FF4444AA";               // 빨강
            }

            for (let key in heatmap) {
                let count = heatmap[key]
                if (count <= 0) continue

                let parts = key.split(",")
                let cx = parseInt(parts[0])
                let cy = parseInt(parts[1])

                // 월드 좌표 셀 영역 좌상단
                let wx = cx * size
                let wy = cy * size

                // 사각형 네 귀퉁이 월드 → 화면 좌표 변환
                let p1 = worldToScreen(wx, wy)
                let p2 = worldToScreen(wx + size, wy)
                let p3 = worldToScreen(wx + size, wy + size)
                let p4 = worldToScreen(wx, wy + size)

                // 화면 셀 너비/높이
                let w = Math.abs(p2.x - p1.x)
                let h = Math.abs(p4.y - p1.y)

                let obj = `
                    import QtQuick 2.15
                    Rectangle {
                        width: ${w}
                        height: ${h}
                        x: ${p1.x}
                        y: ${p1.y}
                        color: "${heatColor(count)}"
                    }
                `
                Qt.createQmlObject(obj, heatmapLayer)
            }
        }


        //------------------------------------
        // 선택된 작업자 Trajectory 그리기
        //------------------------------------
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
                let alpha = (j + 1) / n   // 뒤로 갈수록 진하게

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

        //------------------------------------
        // 레이어들 (그리기 순서)
        //------------------------------------
        // 1) Heatmap
        Item {
            id: heatmapLayer
            anchors.fill: parent
            z: 30
        }

        // 2) Trajectory
        Item {
            id: trajectoryLayer
            anchors.fill: parent
            z: 40
        }

        // 3) 마커 레이어
        Item {
            id: workerLayer
            anchors.fill: parent
            z: 50
        }

        //------------------------------------
        // 전체 작업자 목록 모델
        //------------------------------------
        ListModel {
            id: workerListModel
        }

        //------------------------------------
        // MQTT worker 업데이트 처리
        //------------------------------------
        Connections {
            target: sensorProvider

            function onWorkersUpdated(workers) {
                console.log("🧍 전체 업데이트:", workers.length)

                workerListModel.clear()

                // 마커 제거
                for (let i = workerLayer.children.length - 1; i >= 0; i--)
                    workerLayer.children[i].destroy()

                for (let i = 0; i < workers.length; i++) {
                    let w = workers[i]

                    // 목록 모델 채우기
                    workerListModel.append({
                        "name":   w.name,
                        "trackId": w.track_id,
                        "x":      w.x,
                        "y":      w.y,
                        "helmet": w.helmet,
                        "vest":   w.vest,
                        "safe":   w.safe,
                        "stamp":  w.stamp
                    })


                    // 🔥 Trajectory 저장
                    if (!mapArea.trajectories[w.track_id])
                        mapArea.trajectories[w.track_id] = []

                    mapArea.trajectories[w.track_id].push({ x: w.x, y: w.y })

                    // 오래된 점 제거 (메모리 관리)
                    if (mapArea.trajectories[w.track_id].length > 1000)
                        mapArea.trajectories[w.track_id].shift()

                    // 🔥 Heatmap 누적
                    let cx = Math.floor(w.x / mapArea.cellSize)
                    let cy = Math.floor(w.y / mapArea.cellSize)
                    let key = cx + "," + cy

                    if (!mapArea.heatmap[key])
                        mapArea.heatmap[key] = 0
                    mapArea.heatmap[key] += 1

                    // 마커 그리기
                    let pos = mapArea.worldToScreen(w.x, w.y)

                    let markerQml = `
                        import QtQuick 2.15
                        Item {
                            objectName: "workerMarker"
                            property int trackId: ${w.track_id}
                            property bool safe: ${w.safe}
                            width: 40; height: 40
                            x: ${pos.x - 10};
                            y: ${pos.y - 10};

                            Rectangle {
                                id: dangerPulse
                                anchors.centerIn: parent
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
                                }

                                SequentialAnimation on opacity {
                                    loops: Animation.Infinite
                                    running: !safe
                                    NumberAnimation { from: 0.5; to: 0.0; duration: 1200 }
                                }
                            }

                            Image {
                                    anchors.fill: parent
                                    source: safe
                                        ? "file:///C:/QT_QML/project_QML_1/icons/safe.png"
                                        : "file:///C:/QT_QML/project_QML_1/icons/red.png"

                                    fillMode: Image.PreserveAspectFit
                                    antialiasing: true
                                    smooth: true
                            }

                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    mapArea.selectedWorkerId = trackId
                                    mapArea.updateSelectedFromModel()
                                }
                            }
                        }
                    `
                    Qt.createQmlObject(markerQml, workerLayer)
                }

                // 선택된 작업자가 있으면, 새 데이터로 갱신
                if (mapArea.selectedWorkerId !== -1)
                    mapArea.updateSelectedFromModel()

                // Heatmap / Trajectory 갱신
                //mapArea.drawHeatmap()
                mapArea.drawTrajectory()
            }
        }

        //---------------------------------------------
        // 선택 변경 시 Trajectory 재그리기
        //---------------------------------------------
        Connections {
            target: mapArea
            onSelectedWorkerIdChanged: {
                mapArea.updateSelectedFromModel()
                mapArea.drawTrajectory()
            }
        }

        //---------------------------------------------
        // 우측 상단: 전체 작업자 이름 목록 패널
        //---------------------------------------------
        /*
        Rectangle {
            id: workerListPanel
            width: mapArea.width * 0.18
            height: mapArea.height * 0.5
            anchors.top: mapArea.top
            anchors.right: mapArea.right
            anchors.margins: 10
            radius: 10
            color: "#00000088"
            border.color: "#FFFFFF55"
            z: 99

            Column {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 5

                Text {
                    text: "📋 작업자 목록"
                    color: "yellow"
                    font.pixelSize: 12
                    font.bold: true
                }

                ListView {
                    id: listView
                    model: workerListModel
                    clip: true

                    delegate: Rectangle {
                        width: ListView.view.width
                        height: 30
                        color: (mapArea.selectedWorkerId === trackId)
                               ? "#444466"
                               : "transparent"

                        Row {
                            anchors.verticalCenter: parent.verticalCenter
                            spacing: 6

                            Text {
                                text: name
                                color: "white"
                                font.pixelSize: 13
                                font.bold: (mapArea.selectedWorkerId === trackId)
                            }
                        }

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                mapArea.selectedWorkerId = trackId
                                mapArea.updateSelectedFromModel()
                            }
                        }
                    }
                }
            }
        }
        */

        //---------------------------------------------
        // 선택된 작업자 팝업 (마커 옆)
        //---------------------------------------------
        Rectangle {
            id: popup
            visible: mapArea.selectedWorkerData !== null
            radius: 6
            width: 100
            height: 100
            z: 200

            // 배경
            color: "#333333CC"
            border.color: "black"
            border.width: 1

            // 내부 내용용 배경
            Rectangle {
                anchors.fill: parent
                anchors.margins: 4
                color: "#00000055"
                radius: 4
            }

            function updatePosition() {
                if (!mapArea.selectedWorkerData)
                    return

                let p = mapArea.worldToScreen(
                    mapArea.selectedWorkerData.x,
                    mapArea.selectedWorkerData.y
                )

                x = p.x + 15
                y = p.y - height - 5

                if (x + width > mapArea.width)
                    x = mapArea.width - width - 5
                if (x < 0)
                    x = 5
                if (y < 0)
                    y = p.y + 10
                if (y + height > mapArea.height)
                    y = mapArea.height - height - 5
            }

            Column {
                anchors.fill: parent
                anchors.margins: 6
                spacing: 2

                Text {
                    text: mapArea.selectedWorkerData
                          ? "이름: " + mapArea.selectedWorkerData.name
                          : ""
                    color: "yellow"
                    font.pixelSize: 11
                    font.bold: true
                }
                Text {
                    text: mapArea.selectedWorkerData
                          ? "ID: " + mapArea.selectedWorkerData.trackId
                          : ""
                    color: "black"
                    font.pixelSize: 11
                    font.bold: true
                }
                Text {
                    text: mapArea.selectedWorkerData
                          ? "헬멧: " + (mapArea.selectedWorkerData.helmet ? "착용" : "미착용")
                          : ""
                    color: "black"
                    font.pixelSize: 11
                    font.bold: true
                }
                Text {
                    text: mapArea.selectedWorkerData
                          ? "조끼: " + (mapArea.selectedWorkerData.vest ? "착용" : "미착용")
                          : ""
                    color: "black"
                    font.pixelSize: 11
                    font.bold: true
                }
                Text {
                    text: mapArea.selectedWorkerData
                          ? "상태: " + (mapArea.selectedWorkerData.safe ? "안전" : "위험")
                          : ""
                    color: mapArea.selectedWorkerData && mapArea.selectedWorkerData.safe
                           ? "#00FF00" : "red"
                    font.pixelSize: 11
                    font.bold: true
                }
            }

            Connections {
                target: mapArea
                onSelectedWorkerDataChanged: popup.updatePosition()
            }
        }

}

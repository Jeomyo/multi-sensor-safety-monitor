import QtQuick
import QtQuick.Controls

Item {
    id: mapPage
    anchors.fill: parent

    Text {
        text: "공장 내 작업자 위치"
        anchors.bottom: mapArea.top
        anchors.horizontalCenter: mapArea.horizontalCenter
        anchors.bottomMargin: 10
        color: "white"
        font.pixelSize: 20
    }

    Rectangle {
        id: mapArea
        anchors.centerIn: parent
        width: parent.width * 0.9
        height: parent.height * 0.8
        radius: 10
        color: "#2C2F34"
        border.color: "#666"
        border.width: 2

        property double minX: 0
        property double maxX: 0
        property double minY: 0
        property double maxY: 0
        property double scale: 1
        property double offsetX: 0
        property double offsetY: 0
        property double rotationAngle: 0

        // ✅ 부드러운 회전 애니메이션 추가
        Behavior on rotationAngle {
            NumberAnimation { duration: 200; easing.type: Easing.InOutQuad }
        }

        Item { id: mapLayer; anchors.fill: parent }

        Text {
            id: coordLabel
            text: ""
            color: "white"
            font.pixelSize: 14
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.rightMargin: 10
            anchors.bottomMargin: 10
            visible: false
        }
        // ✅ 장애물도 좌표 기반 회전 반영되도록 수정
        function updateObjectsRotation() {
            for (let i = 0; i < mapLayer.children.length; ++i) {
                let obj = mapLayer.children[i]

                if (obj.objectName === "workerMarker") {
                    let rotated = rotatePoint(obj.x, obj.y)
                    obj.x = rotated.x
                    obj.y = rotated.y
                }

                else if (obj.objectName === "obstacleMarker") {
                    let rotated = rotatePoint(obj.baseX, obj.baseY)
                    obj.x = rotated.x - obj.width / 2
                    obj.y = rotated.y - obj.height / 2
                    obj.rotation = mapArea.rotationAngle + obj.baseAngle
                }
            }
        }

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            hoverEnabled: true
            acceptedButtons: Qt.LeftButton

            property double startX: 0
            property double startAngle: 0

            onPressed: {
                if (mouse.button === Qt.LeftButton) {
                    startX = mouse.x
                    startAngle = mapArea.rotationAngle
                }
            }

            onPositionChanged: function(mouse) {
                if (mouse.buttons & Qt.LeftButton) {
                    let delta = mouse.x - startX
                    mapArea.rotationAngle = startAngle + delta * 0.3

                    for (let i = 0; i < mapLayer.children.length; ++i) {
                        let obj = mapLayer.children[i]
                        if (obj.objectName === "contourCanvas") {
                            obj.rotationAngle = mapArea.rotationAngle
                            obj.requestPaint()
                        }
                    }

                    updateObjectsRotation()
                } else {
                    if (mapArea.scale <= 0) return

                    let cx = mapArea.width / 2
                    let cy = mapArea.height / 2
                    let rad = -mapArea.rotationAngle * Math.PI / 180

                    let dx = mouse.x - cx
                    let dy = mouse.y - cy
                    let rotatedX = Math.cos(rad) * dx - Math.sin(rad) * dy + cx
                    let rotatedY = Math.sin(rad) * dx + Math.cos(rad) * dy + cy

                    let mapX = (rotatedX - mapArea.offsetX) / mapArea.scale + mapArea.minX
                    let mapY = (mapArea.height - rotatedY - mapArea.offsetY) / mapArea.scale + mapArea.minY

                    coordLabel.text = `X: ${mapX.toFixed(2)} , Y: ${mapY.toFixed(2)}`
                    coordLabel.visible = true
                }
            }

            onExited: coordLabel.visible = false
        }

        Connections {
            target: sensorProvider

            function tx(x) { return (x - mapArea.minX) * mapArea.scale + mapArea.offsetX }
            function ty(y) { return mapArea.height - ((y - mapArea.minY) * mapArea.scale + mapArea.offsetY) }

            function rotatePoint(x, y) {
                let cx = mapArea.width / 2
                let cy = mapArea.height / 2
                let rad = mapArea.rotationAngle * Math.PI / 180
                let dx = x - cx
                let dy = y - cy
                return {
                    x: Math.cos(rad) * dx - Math.sin(rad) * dy + cx,
                    y: Math.sin(rad) * dx + Math.cos(rad) * dy + cy
                }
            }

            function onMapUpdated(mapPoints) {
                console.log("✅ 맵 데이터 수신:", mapPoints.length)

                let filtered = []
                const threshold = 0.05
                for (let p of mapPoints) {
                    let dup = false
                    for (let q of filtered) {
                        let dx = p.x - q.x
                        let dy = p.y - q.y
                        if (dx * dx + dy * dy < threshold * threshold) { dup = true; break }
                    }
                    if (!dup) filtered.push(p)
                }
                if (filtered.length < 3) return

                function cross(o,a,b){return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x)}
                filtered.sort((a,b)=>a.x===b.x?a.y-b.y:a.x-b.x)
                let lower=[],upper=[]
                for (let p of filtered){
                    while (lower.length>=2 && cross(lower[lower.length-2],lower[lower.length-1],p)<=0) lower.pop()
                    lower.push(p)
                }
                for (let i=filtered.length-1;i>=0;i--){
                    let p=filtered[i]
                    while (upper.length>=2 && cross(upper[upper.length-2],upper[upper.length-1],p)<=0) upper.pop()
                    upper.push(p)
                }
                upper.pop(); lower.pop()
                let hull = lower.concat(upper)
                if (hull.length < 3) return

                mapArea.minX = Math.min(...hull.map(p => p.x))
                mapArea.maxX = Math.max(...hull.map(p => p.x))
                mapArea.minY = Math.min(...hull.map(p => p.y))
                mapArea.maxY = Math.max(...hull.map(p => p.y))
                let scaleX = mapArea.width / (mapArea.maxX - mapArea.minX)
                let scaleY = mapArea.height / (mapArea.maxY - mapArea.minY)
                mapArea.scale = Math.min(scaleX, scaleY) * 0.9
                let mapW = (mapArea.maxX - mapArea.minX) * mapArea.scale
                let mapH = (mapArea.maxY - mapArea.minY) * mapArea.scale
                mapArea.offsetX = (mapArea.width - mapW) / 2
                mapArea.offsetY = (mapArea.height - mapH) / 2

                for (let i = mapLayer.children.length - 1; i >= 0; i--)
                    mapLayer.children[i].destroy()

                let canvasQml = `
                    import QtQuick 2.15
                    Canvas {
                        objectName: "contourCanvas"
                        width: parent.width; height: parent.height
                        antialiasing: true
                        property var pts: []
                        property double rotationAngle: ${mapArea.rotationAngle}
                        property double cx: width / 2
                        property double cy: height / 2
                        function tx(x) { return (x - ${mapArea.minX}) * ${mapArea.scale} + ${mapArea.offsetX} }
                        function ty(y) { return height - ((y - ${mapArea.minY}) * ${mapArea.scale} + ${mapArea.offsetY}) }
                        onPaint: {
                            if (pts.length < 2) return;
                            var ctx = getContext("2d");
                            ctx.reset();
                            ctx.translate(cx, cy);
                            ctx.rotate(rotationAngle * Math.PI / 180);
                            ctx.translate(-cx, -cy);
                            ctx.lineWidth = 3;
                            ctx.strokeStyle = "#FFFFFF";
                            ctx.beginPath();
                            ctx.moveTo(tx(pts[0].x), ty(pts[0].y));
                            for (var i=1;i<pts.length;i++)
                                ctx.lineTo(tx(pts[i].x), ty(pts[i].y));
                            ctx.closePath();
                            ctx.stroke();
                        }
                    }
                `
                let canvas = Qt.createQmlObject(canvasQml, mapLayer)
                canvas.pts = hull
                canvas.requestPaint()
                console.log("✅ 외곽선 생성 완료:", hull.length, "points")

                loadObstaclesFromJson()
            }

            Component.onCompleted: {
                console.log("🗺️ contour.json 로컬 파일 로드 시작...")
                var xhr = new XMLHttpRequest()
                xhr.open("GET", "file:///C:/QT_QML/project_QML_1/contour.json")
                xhr.onreadystatechange = function() {
                    if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
                        try {
                            let points = JSON.parse(xhr.responseText)
                            onMapUpdated(points)
                        } catch (e) {
                            console.error("❌ JSON 파싱 실패:", e)
                        }
                    }
                }
                xhr.send()
            }

            function loadObstaclesFromJson() {
                console.log("🧱 obstacles.json 로드 시작...")
                var xhr = new XMLHttpRequest()
                xhr.open("GET", "file:///C:/QT_QML/project_QML_1/obstacle.json")
                xhr.onreadystatechange = function() {
                    if (xhr.readyState === XMLHttpRequest.DONE && xhr.status === 200) {
                        try {
                            let obstacles = JSON.parse(xhr.responseText)
                            drawObstacles(obstacles)
                            console.log("✅ 장애물 데이터 로드 완료:", obstacles.length)
                        } catch (e) {
                            console.error("❌ 장애물 JSON 파싱 실패:", e)
                        }
                    }
                }
                xhr.send()
            }

            function drawObstacles(obstacles) {
                for (let i = mapLayer.children.length - 1; i >= 0; i--) {
                    if (mapLayer.children[i].objectName === "obstacleMarker")
                        mapLayer.children[i].destroy()
                }

                for (let i = 0; i < obstacles.length; ++i) {
                    let o = obstacles[i]
                    let px = tx(o.x)
                    let py = ty(o.y)
                    let rotated = rotatePoint(px, py)

                    let imgQml = `
                        import QtQuick 2.15
                        Image {
                            objectName: "obstacleMarker"
                            property double baseX: ${px}
                            property double baseY: ${py}
                            property double baseAngle: ${o.angle || 0}
                            property double widthScale: ${o.width * mapArea.scale}
                            property double heightScale: ${o.height * mapArea.scale}
                            source: "${o.image}"
                            width: widthScale
                            height: heightScale
                            smooth: true
                            antialiasing: true

                            function updatePosition() {
                                let cx = ${mapArea.width / 2}
                                let cy = ${mapArea.height / 2}
                                let rad = mapArea.rotationAngle * Math.PI / 180
                                let dx = baseX - cx
                                let dy = baseY - cy
                                let newX = Math.cos(rad) * dx - Math.sin(rad) * dy + cx
                                let newY = Math.sin(rad) * dx + Math.cos(rad) * dy + cy
                                x = newX - width / 2
                                y = newY - height / 2
                                rotation = mapArea.rotationAngle + baseAngle
                            }

                            Connections {
                                target: mapArea
                                onRotationAngleChanged: updatePosition()   // ✅ 회전 시 실시간 반영
                            }

                            Component.onCompleted: updatePosition()
                        }
                    `
                    Qt.createQmlObject(imgQml, mapLayer)
                }
            }

            // ✅ 🧍 작업자 표시 (MQTT 수신 시)
            function onWorkersUpdated(workers) {
                console.log("🧍 작업자 수신:", workers.length)

                for (let i = mapLayer.children.length - 1; i >= 0; i--) {
                    if (mapLayer.children[i].objectName === "workerMarker")
                        mapLayer.children[i].destroy()
                }

                for (let i = 0; i < workers.length; ++i) {
                    let w = workers[i]
                    let color = w.safe ? "#00FF00" : "red"

                    let px = tx(w.x)
                    let py = ty(w.y)
                    let rotated = rotatePoint(px, py)

                    let item = Qt.createQmlObject(
                        'import QtQuick 2.15; Item { objectName: "workerMarker"; width: 18; height: 18 }',
                        mapLayer
                    )
                    item.x = rotated.x
                    item.y = rotated.y

                    Qt.createQmlObject(
                        'import QtQuick 2.15; Rectangle { width: 18; height: 18; radius: 9; color: "' + color + '"; anchors.centerIn: parent }',
                        item
                    )

                    Qt.createQmlObject(
                        'import QtQuick 2.15; Text { text: "' + w.name + ' (' + (w.safe ? "착용" : "미착용") + ')"; color: "white"; font.pixelSize: 12; anchors.left: parent.right; anchors.leftMargin: 6 }',
                        item
                    )
                }
            }
        }
    }
}

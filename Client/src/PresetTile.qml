import QtQuick
import RoadPainter

// 클릭하면 즉시 중앙 프리셋 생성. (드래그해 TopView에 놓기도 가능)
Item {
    id: tile
    width: 52
    height: 64
    property string shape: "RECT"
    property string label: ""
    property Item rootItem: null

    Rectangle {
        id: content
        width: 52
        height: 48
        radius: Theme.radius
        property string shape: tile.shape
        color: dragArea.drag.active ? Theme.accentSoft : Theme.elevated
        border.color: dragArea.containsMouse || dragArea.drag.active ? Theme.accent : Theme.stroke
        border.width: 1

        Drag.active: dragArea.drag.active
        Drag.hotSpot: Qt.point(26, 24)
        Drag.keys: ["preset-shape"]

        Canvas {
            id: cv
            anchors.fill: parent
            anchors.margins: 8
            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                var w = width, h = height
                ctx.lineWidth = 2
                ctx.strokeStyle = Theme.text
                ctx.fillStyle = Theme.accentSoft
                ctx.lineJoin = "round"
                var s = tile.shape
                ctx.beginPath()
                if (s === "RECT") {
                    ctx.rect(2, 6, w - 4, h - 12)
                } else if (s === "TRIANGLE") {
                    ctx.moveTo(w / 2, 2); ctx.lineTo(w - 2, h - 2); ctx.lineTo(2, h - 2); ctx.closePath()
                } else if (s === "CIRCLE") {
                    ctx.ellipse(2, 4, w - 4, h - 8)
                } else if (s === "CROSSWALK") {
                    // 세로 줄 4개
                    var bw = (w - 6) / 7
                    for (var i = 0; i < 4; ++i)
                        ctx.rect(3 + i * 2 * bw, 3, bw, h - 6)
                } else if (s === "STOPLINE") {
                    ctx.rect(2, h / 2 - 5, w - 4, 10)
                } else if (s === "ARROW_F") {
                    // 실제 작도와 같이 "기둥 획 + 화살촉 획" (외곽선 아님)
                    ctx.moveTo(w / 2, h - 2); ctx.lineTo(w / 2, 4)
                    ctx.moveTo(w / 2 - 8, 12); ctx.lineTo(w / 2, 4); ctx.lineTo(w / 2 + 8, 12)
                    ctx.lineWidth = 3.5
                    ctx.stroke()
                    return
                } else if (s === "ARROW_L" || s === "ARROW_R") {
                    var m = (s === "ARROW_L") ? 1 : -1
                    var cx0 = w / 2 + m * (w / 5)
                    var hx = w / 2 - m * (w / 3)
                    // 기둥: 직진 → 회전 → 직진
                    ctx.moveTo(cx0, h - 2)
                    ctx.lineTo(cx0, h / 2)
                    ctx.lineTo(hx, h / 2)
                    // 화살촉: 촉 끝에서 양쪽으로 (한 획)
                    ctx.moveTo(hx + m * 8, h / 2 - 7)
                    ctx.lineTo(hx, h / 2)
                    ctx.lineTo(hx + m * 8, h / 2 + 7)
                    ctx.lineWidth = 3.5
                    ctx.stroke()
                    return
                } else if (s === "PARKING") {
                    ctx.lineWidth = 3
                    ctx.moveTo(6, 2); ctx.lineTo(6, h - 3)
                    ctx.lineTo(w - 6, h - 3); ctx.lineTo(w - 6, 2)
                    ctx.stroke()
                    return
                } else if (s === "ZIGZAG") {
                    ctx.moveTo(2, h - 4)
                    ctx.lineTo(w * 0.25, 4)
                    ctx.lineTo(w * 0.5, h - 4)
                    ctx.lineTo(w * 0.75, 4)
                    ctx.lineTo(w - 2, h - 4)
                    ctx.stroke()
                    return
                } else { // LINE
                    ctx.moveTo(2, h / 2); ctx.lineTo(w - 2, h / 2)
                }
                if (s !== "LINE") ctx.fill()
                ctx.stroke()
            }
            Component.onCompleted: requestPaint()
        }

        MouseArea {
            id: dragArea
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            drag.target: content
            drag.threshold: 8
            onClicked: Backend.addPreset(tile.shape)
            onReleased: {
                if (content.Drag.active)
                    content.Drag.drop()
            }
        }

        states: State {
            when: dragArea.drag.active
            ParentChange { target: content; parent: tile.rootItem ? tile.rootItem : tile }
        }
    }

    Text {
        anchors.top: content.bottom
        anchors.topMargin: 2
        anchors.horizontalCenter: content.horizontalCenter
        text: tile.label
        color: Theme.sub
        font.pixelSize: 10
        font.family: Theme.fontFamily
    }
}

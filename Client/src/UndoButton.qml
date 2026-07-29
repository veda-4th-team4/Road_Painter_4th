import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

// 실행 취소(Ctrl+Z) 아이콘 버튼 — 원형 '돌아가기 화살표' 그림(Canvas)
Item {
    id: root
    width: 42
    height: 42
    signal clicked()
    property bool hovered: ma.containsMouse

    Rectangle {
        anchors.fill: parent
        radius: width / 2
        color: ma.pressed ? Theme.accentDim : (root.hovered ? Theme.accent : Theme.elevated)
        border.color: root.hovered ? Theme.accent : Theme.stroke
        border.width: 1

        Canvas {
            id: cv
            anchors.fill: parent
            property color col: (root.hovered || ma.pressed) ? "#FFFFFF" : Theme.text
            onColChanged: requestPaint()
            onPaint: {
                var ctx = getContext("2d")
                ctx.reset()
                var cx = width / 2, cy = height / 2 + 1, r = 9
                ctx.strokeStyle = col
                ctx.fillStyle = col
                ctx.lineWidth = 2.2
                ctx.lineCap = "round"

                // 위를 도는 반원 호 (반시계)
                ctx.beginPath()
                ctx.arc(cx, cy, r, 0, Math.PI, true)
                ctx.stroke()

                // 좌측 끝에 아래로 향하는 화살촉
                var tx = cx - r, ty = cy
                ctx.beginPath()
                ctx.moveTo(tx, ty + 5)
                ctx.lineTo(tx - 4.5, ty - 3)
                ctx.lineTo(tx + 4.5, ty - 3)
                ctx.closePath()
                ctx.fill()
            }
        }
    }

    MouseArea {
        id: ma
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }

    ToolTip.visible: root.hovered
    ToolTip.text: "실행 취소 (Ctrl+Z)"
}

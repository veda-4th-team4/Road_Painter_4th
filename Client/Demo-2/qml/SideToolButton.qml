import QtQuick
import RoadPainter

// 좌측 툴바 버튼: Canvas로 그린 아이콘 + 짧은 라벨.
Item {
    id: root

    property string iconName: "cursor"   // cursor | pen | measure | preset | trash
    property string label: ""
    property bool checked: false
    property bool danger: false
    signal clicked()

    width: 52
    height: 56

    Rectangle {
        anchors.fill: parent
        radius: 9
        color: root.checked ? Qt.alpha(Theme.accent, 0.16)
             : mouseArea.containsMouse ? Theme.bg3 : "transparent"
        border.color: root.checked ? Qt.alpha(Theme.accent, 0.55) : "transparent"
        border.width: 1
    }

    Canvas {
        id: icon
        width: 20
        height: 20
        anchors.horizontalCenter: parent.horizontalCenter
        y: 9

        property color ink: root.danger && mouseArea.containsMouse ? Theme.danger
                          : root.checked ? Theme.accent
                          : mouseArea.containsMouse ? Theme.textHi : Theme.textMid
        onInkChanged: requestPaint()

        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            ctx.strokeStyle = ink
            ctx.fillStyle = ink
            ctx.lineWidth = 1.7
            ctx.lineCap = "round"
            ctx.lineJoin = "round"

            if (root.iconName === "cursor") {
                ctx.beginPath()
                ctx.moveTo(5, 3)
                ctx.lineTo(5, 16)
                ctx.lineTo(8.5, 12.5)
                ctx.lineTo(11, 17.5)
                ctx.lineTo(13.2, 16.4)
                ctx.lineTo(10.7, 11.5)
                ctx.lineTo(15.5, 11)
                ctx.closePath()
                ctx.fill()
            } else if (root.iconName === "pen") {
                ctx.beginPath()
                ctx.moveTo(3.5, 16.5)
                ctx.lineTo(5, 12.5)
                ctx.lineTo(13.5, 4)
                ctx.lineTo(16, 6.5)
                ctx.lineTo(7.5, 15)
                ctx.closePath()
                ctx.stroke()
                ctx.beginPath()
                ctx.moveTo(3.5, 16.5)
                ctx.lineTo(7.5, 15)
                ctx.stroke()
            } else if (root.iconName === "measure") {
                ctx.beginPath()
                ctx.arc(10, 10, 5.5, 0, Math.PI * 2)
                ctx.stroke()
                ctx.beginPath()
                ctx.moveTo(10, 1.5); ctx.lineTo(10, 5.5)
                ctx.moveTo(10, 14.5); ctx.lineTo(10, 18.5)
                ctx.moveTo(1.5, 10); ctx.lineTo(5.5, 10)
                ctx.moveTo(14.5, 10); ctx.lineTo(18.5, 10)
                ctx.stroke()
                ctx.beginPath()
                ctx.arc(10, 10, 1.2, 0, Math.PI * 2)
                ctx.fill()
            } else if (root.iconName === "preset") {
                ctx.strokeRect(3, 3, 8.5, 8.5)
                ctx.beginPath()
                ctx.moveTo(13, 10)
                ctx.lineTo(17.5, 17.5)
                ctx.lineTo(8.5, 17.5)
                ctx.closePath()
                ctx.stroke()
            } else if (root.iconName === "trash") {
                ctx.beginPath()
                ctx.moveTo(4, 6); ctx.lineTo(16, 6)
                ctx.moveTo(8, 6); ctx.lineTo(8.5, 3.5); ctx.lineTo(11.5, 3.5); ctx.lineTo(12, 6)
                ctx.stroke()
                ctx.beginPath()
                ctx.moveTo(5.5, 6); ctx.lineTo(6.5, 17); ctx.lineTo(13.5, 17); ctx.lineTo(14.5, 6)
                ctx.stroke()
                ctx.beginPath()
                ctx.moveTo(8.4, 8.5); ctx.lineTo(8.8, 14.5)
                ctx.moveTo(11.6, 8.5); ctx.lineTo(11.2, 14.5)
                ctx.stroke()
            }
        }
    }

    Text {
        text: root.label
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 5
        font.pixelSize: Theme.fontXs
        color: icon.ink
    }

    MouseArea {
        id: mouseArea
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: root.clicked()
    }
}

import QtQuick
import RoadPainter

// 축척 눈금자. 화면에 박아두지 않고 사용자가 끌어서 옮길 수 있다.
// 놓으면 가장 가까운 모서리로 붙는다(스냅) — 경로를 가리면 반대쪽으로 치우면 됨.
Item {
    id: ruler

    property real pxPerMm: 0        // 화면 1mm 가 몇 px 인가 (표시 배율 반영)
    property real mmPerPx: 0        // 보정 1px = ? mm (툴팁용)
    property int  margin: 10
    property bool moved: false      // 사용자가 한 번이라도 옮겼는가

    readonly property var steps: [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
    readonly property real barMm: {
        if (pxPerMm <= 0) return 100
        for (let i = 0; i < steps.length; ++i)
            if (steps[i] * pxPerMm >= 68) return steps[i]
        return steps[steps.length - 1]
    }
    readonly property real barPx: Math.max(8, barMm * pxPerMm)
    readonly property string barLabel: barMm >= 1000
        ? (barMm / 1000).toFixed(barMm % 1000 === 0 ? 0 : 2) + " m"
        : barMm + " mm"

    visible: pxPerMm > 0
    width: box.width
    height: box.height
    z: 5

    // 기본 자리는 우하단 — 바인딩이라 패널 크기가 바뀌어도 따라간다.
    // 사용자가 끌면 drag 가 x/y 에 값을 써서 바인딩이 끊기고, 그 자리를 지킨다.
    x: parent ? Math.max(margin, parent.width - width - margin) : margin
    y: parent ? Math.max(margin, parent.height - height - margin) : margin

    // 옮겨둔 뒤 패널이 줄어들어도 밖으로 나가지 않게
    onMovedChanged: keepInside()
    function keepInside() {
        if (!parent || !moved) return
        x = Math.max(margin, Math.min(x, parent.width - width - margin))
        y = Math.max(margin, Math.min(y, parent.height - height - margin))
    }
    Connections {
        target: ruler.moved ? ruler.parent : null
        ignoreUnknownSignals: true
        function onWidthChanged() { ruler.keepInside() }
        function onHeightChanged() { ruler.keepInside() }
    }

    Rectangle {
        id: box
        width: content.implicitWidth + 16
        height: 28
        radius: 6
        color: drag.pressed ? "#E61A1D21" : "#B31A1D21"
        border.width: drag.containsMouse || drag.pressed ? 1 : 0
        border.color: Theme.accent

        Row {
            id: content
            anchors.centerIn: parent
            spacing: 8

            // 눈금자: ├─────┤
            Item {
                width: ruler.barPx
                height: 12
                anchors.verticalCenter: parent.verticalCenter
                Rectangle { anchors.verticalCenter: parent.verticalCenter; width: parent.width; height: 2; color: "#FFFFFF" }
                Rectangle { x: 0; anchors.verticalCenter: parent.verticalCenter; width: 2; height: 10; color: "#FFFFFF" }
                Rectangle { x: parent.width - 2; anchors.verticalCenter: parent.verticalCenter; width: 2; height: 10; color: "#FFFFFF" }
            }
            Text {
                text: ruler.barLabel
                color: "#FFFFFF"
                font.pixelSize: 10
                font.bold: true
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        MouseArea {
            id: drag
            anchors.fill: parent
            hoverEnabled: true
            cursorShape: Qt.SizeAllCursor
            drag.target: ruler
            drag.minimumX: 0
            drag.minimumY: 0
            drag.maximumX: ruler.parent ? ruler.parent.width - ruler.width : 0
            drag.maximumY: ruler.parent ? ruler.parent.height - ruler.height : 0
            onPressed: ruler.moved = true
            onReleased: {
                // 가까운 모서리로 스냅 — 대충 놓아도 정돈되게
                if (!ruler.parent) return
                const cx = ruler.x + ruler.width / 2
                const cy = ruler.y + ruler.height / 2
                ruler.x = (cx < ruler.parent.width / 2)
                    ? ruler.margin : ruler.parent.width - ruler.width - ruler.margin
                ruler.y = (cy < ruler.parent.height / 2)
                    ? ruler.margin : ruler.parent.height - ruler.height - ruler.margin
            }
        }
    }

    Behavior on x { enabled: !drag.pressed; NumberAnimation { duration: 120; easing.type: Easing.OutCubic } }
    Behavior on y { enabled: !drag.pressed; NumberAnimation { duration: 120; easing.type: Easing.OutCubic } }
}

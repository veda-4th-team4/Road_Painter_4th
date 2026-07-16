import QtQuick
import RoadPainter

// 영상 위 경로 작도/편집 오버레이. 좌표계는 스트림 이미지 픽셀(1280x720)과 동일.
//  - draw 모드: 클릭으로 정점 추가, 우클릭으로 완료
//  - select 모드: 정점 드래그 이동, 정점 우클릭 삭제,
//                 바운딩 박스 드래그로 전체 이동, 우하단 핸들로 크기 조절
//  - measure 모드: 십자선 + 좌표 표시, 클릭 시 마커 고정
Item {
    id: root

    property string mode: "select"
    property var model
    property point cursorPos: Qt.point(-1, -1)
    signal finishRequested()

    property rect bbox: Qt.rect(0, 0, 0, 0)
    readonly property bool hasPath: model && model.count > 0
    readonly property bool editVisible: mode === "select" && model && model.count > 1

    onCursorPosChanged: rubber.requestPaint()

    function refresh() {
        if (model)
            bbox = model.boundingBox()
        pathCanvas.requestPaint()
    }

    Component.onCompleted: refresh()

    Connections {
        target: root.model
        function onPathChanged() { root.refresh() }
        function onClosedChanged() { pathCanvas.requestPaint() }
    }

    // ── 경로 폴리라인 ─────────────────────────────────
    Canvas {
        id: pathCanvas
        anchors.fill: parent

        function drawArrow(ctx, a, b) {
            const dx = b.x - a.x
            const dy = b.y - a.y
            const len = Math.hypot(dx, dy)
            if (len < 40)
                return
            const mx = a.x + dx / 2
            const my = a.y + dy / 2
            const ux = dx / len
            const uy = dy / len
            ctx.beginPath()
            ctx.moveTo(mx + ux * 6, my + uy * 6)
            ctx.lineTo(mx - ux * 4 - uy * 4.5, my - uy * 4 + ux * 4.5)
            ctx.lineTo(mx - ux * 4 + uy * 4.5, my - uy * 4 - ux * 4.5)
            ctx.closePath()
            ctx.fill()
        }

        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            if (!root.model || root.model.count === 0)
                return
            const pts = root.model.points()
            const closed = root.model.closed && pts.length > 2

            ctx.lineJoin = "round"
            ctx.lineCap = "round"
            ctx.beginPath()
            ctx.moveTo(pts[0].x, pts[0].y)
            for (let i = 1; i < pts.length; ++i)
                ctx.lineTo(pts[i].x, pts[i].y)
            if (closed)
                ctx.closePath()

            // 글로우 + 본선
            ctx.strokeStyle = Qt.alpha(Theme.pathLine, 0.22)
            ctx.lineWidth = 8
            ctx.stroke()
            ctx.strokeStyle = Theme.pathLine
            ctx.lineWidth = 2.5
            ctx.stroke()

            // 진행 방향 화살표
            ctx.fillStyle = Theme.pathLine
            for (let i = 1; i < pts.length; ++i)
                drawArrow(ctx, pts[i - 1], pts[i])
            if (closed)
                drawArrow(ctx, pts[pts.length - 1], pts[0])
        }
    }

    // ── 작도 중 러버 밴드 ─────────────────────────────
    Canvas {
        id: rubber
        anchors.fill: parent
        visible: root.mode === "draw" && root.hasPath && root.cursorPos.x >= 0
        onVisibleChanged: requestPaint()
        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            if (!visible || !root.model || root.model.count === 0)
                return
            const pts = root.model.points()
            const a = pts[pts.length - 1]
            ctx.setLineDash([7, 7])
            ctx.strokeStyle = Qt.alpha(Theme.pathLine, 0.55)
            ctx.lineWidth = 1.5
            ctx.beginPath()
            ctx.moveTo(a.x, a.y)
            ctx.lineTo(root.cursorPos.x, root.cursorPos.y)
            ctx.stroke()
        }
    }

    // ── 기본 입력 (정점 추가/측정/완료) ────────────────
    MouseArea {
        id: baseArea
        anchors.fill: parent
        hoverEnabled: true
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        cursorShape: root.mode === "draw" || root.mode === "measure"
                     ? Qt.CrossCursor : Qt.ArrowCursor
        onPositionChanged: (m) => root.cursorPos = Qt.point(Math.round(m.x), Math.round(m.y))
        onExited: root.cursorPos = Qt.point(-1, -1)
        onPressed: (m) => {
            if (m.button === Qt.RightButton) {
                if (root.mode === "draw" && root.model.count > 0)
                    root.finishRequested()
                return
            }
            if (root.mode === "draw") {
                root.model.addPoint(m.x, m.y)
            } else if (root.mode === "measure") {
                measurePin.px = m.x
                measurePin.py = m.y
                measurePin.active = true
            }
        }
    }

    // ── 전체 이동 (바운딩 박스) ────────────────────────
    Item {
        id: bodyDrag
        visible: root.editVisible
        x: root.bbox.x
        y: root.bbox.y
        width: root.bbox.width
        height: root.bbox.height

        Rectangle {
            anchors.fill: parent
            anchors.margins: -10
            color: Qt.alpha(Theme.accent, 0.05)
            border.color: Qt.alpha(Theme.accent, 0.5)
            border.width: 1
            radius: 4
        }

        MouseArea {
            anchors.fill: parent
            anchors.margins: -10
            cursorShape: Qt.SizeAllCursor
            property point pressPos
            onPressed: (m) => pressPos = Qt.point(m.x, m.y)
            onPositionChanged: (m) => {
                if (pressed)
                    root.model.translateAll(m.x - pressPos.x, m.y - pressPos.y)
            }
        }
    }

    // ── 크기 조절 핸들 ────────────────────────────────
    Rectangle {
        visible: root.editVisible
        x: root.bbox.x + root.bbox.width + 12
        y: root.bbox.y + root.bbox.height + 12
        width: 14
        height: 14
        radius: 3
        color: scaleMa.pressed ? Qt.lighter(Theme.accent, 1.15) : Theme.accent

        MouseArea {
            id: scaleMa
            anchors.fill: parent
            anchors.margins: -6
            cursorShape: Qt.SizeFDiagCursor
            property real startDist: 1
            property point center
            onPressed: (m) => {
                center = Qt.point(root.bbox.x + root.bbox.width / 2,
                                  root.bbox.y + root.bbox.height / 2)
                const p = mapToItem(root, m.x, m.y)
                startDist = Math.max(12, Math.hypot(p.x - center.x, p.y - center.y))
            }
            onPositionChanged: (m) => {
                if (!pressed)
                    return
                const p = mapToItem(root, m.x, m.y)
                const d = Math.max(12, Math.hypot(p.x - center.x, p.y - center.y))
                root.model.scaleAround(center.x, center.y, d / startDist)
                startDist = d
            }
        }
    }

    // ── 정점 핸들 ─────────────────────────────────────
    Repeater {
        model: root.model

        delegate: Item {
            id: handle
            required property int index
            required property real px
            required property real py
            x: px
            y: py
            visible: root.mode === "select" || root.mode === "draw"

            Rectangle {
                x: -6; y: -6
                width: 12; height: 12
                radius: 6
                color: handleMa.pressed ? Theme.accent
                     : handleMa.containsMouse ? Theme.bg3 : "#10131a"
                border.color: handle.index === 0 ? Theme.ok : Theme.pathLine
                border.width: 2
            }

            Text {
                visible: handleMa.containsMouse
                x: 10; y: -22
                text: (handle.index + 1) + " · (" + Math.round(handle.px) + ", " + Math.round(handle.py) + ")"
                color: Theme.textHi
                font.pixelSize: Theme.fontXs
                font.family: Theme.mono
                style: Text.Outline
                styleColor: "#000000"
            }

            MouseArea {
                id: handleMa
                x: -11; y: -11
                width: 22; height: 22
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton | Qt.RightButton
                cursorShape: Qt.PointingHandCursor
                onPositionChanged: (m) => {
                    if (pressed && (m.buttons & Qt.LeftButton)) {
                        const p = mapToItem(root, m.x, m.y)
                        root.model.setPoint(handle.index, p.x, p.y)
                    }
                }
                onClicked: (m) => {
                    if (m.button === Qt.RightButton)
                        root.model.removeAt(handle.index)
                }
            }
        }
    }

    // ── 측정 모드 십자선 ──────────────────────────────
    Item {
        anchors.fill: parent
        visible: root.mode === "measure" && root.cursorPos.x >= 0

        Rectangle {
            x: root.cursorPos.x
            width: 1
            height: parent.height
            color: Qt.alpha(Theme.ok, 0.55)
        }
        Rectangle {
            y: root.cursorPos.y
            width: parent.width
            height: 1
            color: Qt.alpha(Theme.ok, 0.55)
        }
        Rectangle {
            x: Math.min(root.cursorPos.x + 14, parent.width - width - 4)
            y: Math.max(root.cursorPos.y - 30, 4)
            width: coordLabel.width + 14
            height: coordLabel.height + 8
            radius: 4
            color: "#0c1210ee"
            border.color: Qt.alpha(Theme.ok, 0.6)

            Text {
                id: coordLabel
                anchors.centerIn: parent
                text: "X " + root.cursorPos.x + " · Y " + root.cursorPos.y
                color: Theme.ok
                font.pixelSize: Theme.fontSm
                font.family: Theme.mono
            }
        }
    }

    // ── 측정 고정 마커 ────────────────────────────────
    Item {
        id: measurePin
        property real px: 0
        property real py: 0
        property bool active: false
        visible: active && root.mode === "measure"
        x: px
        y: py

        Rectangle {
            x: -5; y: -5
            width: 10; height: 10; radius: 5
            color: "transparent"
            border.color: Theme.warn
            border.width: 2
        }
        Rectangle { x: -1; y: -1; width: 2; height: 2; color: Theme.warn }
        Text {
            x: 10; y: 6
            text: "(" + Math.round(measurePin.px) + ", " + Math.round(measurePin.py) + ")"
            color: Theme.warn
            font.pixelSize: Theme.fontSm
            font.family: Theme.mono
            style: Text.Outline
            styleColor: "#000000"
        }
    }
}

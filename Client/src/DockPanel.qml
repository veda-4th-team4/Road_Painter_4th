import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Window
import RoadPainter

// VSCode 식 도킹 패널.
//   · 헤더를 잡고 밖으로 끌면 별도 창으로 빠진다 (팝아웃)
//   · 떠 있는 창의 헤더를 잡아 원래 자리(점선 영역) 위에서 놓으면 다시 들어간다
//   · ⧉ / ⤓ 버튼으로도 전환 가능, 창을 닫아도 제자리로 복귀
//
// 사용법:
//   Item { id: someSlot }                 // SplitView 안의 빈 자리
//   DockPanel { dock: someSlot; title: "..."; <내용> }
Item {
    id: root

    property Item dock: null              // 도킹될 자리(Item)
    property string title: ""
    property string subtitle: ""
    property bool floating: false
    property int floatWidth: 900
    property int floatHeight: 620
    property bool detachable: true
    default property alias content: contentArea.data

    // 헤더 우측에 끼워 넣을 추가 컨트롤
    property alias headerExtra: extraRow.data

    function popOut(gx, gy) {
        if (!detachable || floating) return
        win.width = floatWidth
        win.height = floatHeight
        // 화면 밖으로 나가 "사라진 것처럼" 보이지 않게 항상 화면 안으로 클램프
        var px = (gx !== undefined) ? gx - 120 : 200
        var py = (gy !== undefined) ? gy - 16 : 160
        px = Math.max(0, Math.min(px, Screen.desktopAvailableWidth - Math.min(win.width, 420)))
        py = Math.max(0, Math.min(py, Screen.desktopAvailableHeight - Math.min(win.height, 320)))
        win.x = px
        win.y = py
        floating = true
        win.show()
        win.raise()
        win.requestActivate()
    }
    function dockBack() {
        if (!floating) return
        floating = false
        win.hide()
    }
    // 전역 좌표가 도킹 자리 위인가
    function isOverDock(gx, gy) {
        if (!dock) return false
        const p = dock.mapToGlobal(0, 0)
        return gx >= p.x && gx <= p.x + dock.width
            && gy >= p.y && gy <= p.y + dock.height
    }

    Rectangle {
        id: frame
        parent: root.floating ? floatHost : (root.dock ? root.dock : root)
        anchors.fill: parent
        radius: Theme.radius
        color: Theme.surface
        border.width: 1
        border.color: root.floating ? Theme.accent : Theme.stroke
        clip: true

        Column {
            anchors.fill: parent
            anchors.margins: 1
            spacing: 0

            // ── 헤더 (드래그 손잡이) ──────────────────────────────
            Rectangle {
                id: header
                width: parent.width
                height: 30
                color: headerMouse.containsMouse ? Theme.panel : Theme.surface
                Rectangle {
                    anchors.bottom: parent.bottom
                    width: parent.width; height: 1; color: Theme.stroke
                }

                Row {
                    anchors.left: parent.left
                    anchors.leftMargin: 10
                    // 제목이 길면 헤더 버튼(확대/축소 등) 위로 넘어와 겹쳤다.
                    // 오른쪽을 버튼 줄에 묶고 잘라내서 다시는 겹치지 않게 한다.
                    anchors.right: extraRow.left
                    anchors.rightMargin: 8
                    clip: true
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 6
                    // 잡는 곳임을 알리는 그립 점
                    Column {
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 2
                        visible: root.detachable
                        Repeater {
                            model: 3
                            Rectangle { width: 10; height: 1; color: Theme.muted; opacity: 0.7 }
                        }
                    }
                    Text {
                        text: root.title
                        color: Theme.sub
                        font.pixelSize: 12
                        font.bold: true
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: root.subtitle
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                        visible: text.length > 0
                    }
                }

                Row {
                    id: extraRow
                    anchors.right: dockBtn.left
                    anchors.rightMargin: 6
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 6
                    // 드래그 영역(headerMouse)보다 위에 있어야 버튼이 클릭을 받는다
                    z: 2
                }

                Rectangle {
                    id: dockBtn
                    anchors.right: parent.right
                    anchors.rightMargin: 6
                    anchors.verticalCenter: parent.verticalCenter
                    z: 2
                    width: 22; height: 22; radius: 4
                    visible: root.detachable
                    color: btnMouse.containsMouse ? Theme.elevated : "transparent"
                    Text {
                        anchors.centerIn: parent
                        text: root.floating ? "⤓" : "⧉"
                        color: Theme.sub
                        font.pixelSize: 12
                        font.family: Theme.fontFamily
                    }
                    MouseArea {
                        id: btnMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        ToolTip.visible: containsMouse
                        ToolTip.text: root.floating ? "제자리로 되돌리기" : "별도 창으로 분리"
                        onClicked: root.floating ? root.dockBack() : root.popOut()
                    }
                }

                MouseArea {
                    id: headerMouse
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    // 헤더에 끼운 컨트롤(지우기·배율 칩 등) 위로는 절대 겹치지 않게 한다.
                    // 겹치면 그 버튼을 눌러도 드래그 영역이 클릭을 삼켜서
                    // "지우기를 눌렀는데 창이 분리되는" 증상이 난다.
                    anchors.right: extraRow.left
                    anchors.rightMargin: 4
                    z: 1
                    hoverEnabled: true
                    enabled: root.detachable
                    cursorShape: root.floating ? Qt.SizeAllCursor : Qt.OpenHandCursor
                    property real pressGX: 0
                    property real pressGY: 0
                    property real grabDX: 0
                    property real grabDY: 0
                    property bool wantDetach: false

                    onPressed: function(m) {
                        const g = mapToGlobal(m.x, m.y)
                        pressGX = g.x; pressGY = g.y
                        wantDetach = false
                        if (root.floating) { grabDX = win.x - g.x; grabDY = win.y - g.y }
                    }
                    onPositionChanged: function(m) {
                        if (!pressed) return
                        const g = mapToGlobal(m.x, m.y)
                        if (!root.floating) {
                            // 드래그 중에는 고스트만 보여주고, 놓는 순간 창을 만든다.
                            // (드래그 도중 reparent 하면 좌표계가 바뀌어 창이 튕겨나간다)
                            if (Math.abs(g.x - pressGX) > 26 || Math.abs(g.y - pressGY) > 26)
                                wantDetach = true
                            if (wantDetach) {
                                ghost.x = g.x + 12
                                ghost.y = g.y + 12
                                if (!ghost.visible) ghost.show()
                                // 도킹 자리 위로 돌아오면 "취소" 라는 걸 고스트 색으로 표시
                                ghost.overDock = root.isOverDock(g.x, g.y)
                            }
                        } else {
                            win.x = g.x + grabDX
                            win.y = g.y + grabDY
                            dropHint.armed = root.isOverDock(g.x, g.y)
                        }
                    }
                    onReleased: function(m) {
                        const g = mapToGlobal(m.x, m.y)
                        if (!root.floating) {
                            ghost.hide()
                            if (wantDetach && !root.isOverDock(g.x, g.y))
                                root.popOut(g.x, g.y)
                            wantDetach = false
                            return
                        }
                        dropHint.armed = false
                        if (root.isOverDock(g.x, g.y))
                            root.dockBack()
                    }
                    // 더블클릭으로는 "되돌리기"만. 실수로 창이 튀어나오는 걸 막는다
                    // (분리는 ⧉ 버튼이나 명시적인 드래그로만)
                    onDoubleClicked: if (root.floating) root.dockBack()
                }
            }

            // ── 내용 ────────────────────────────────────────────
            Item {
                id: contentArea
                width: parent.width
                height: parent.height - header.height
            }
        }
    }

    // 빠져나갔을 때 원래 자리에 남는 드롭 존
    Rectangle {
        id: dropHint
        parent: root.dock ? root.dock : root
        anchors.fill: parent
        visible: root.floating
        z: -1
        property bool armed: false
        radius: Theme.radius
        color: armed ? Theme.accentSoft : "transparent"
        border.width: 2
        border.color: armed ? Theme.accent : Theme.stroke
        Column {
            anchors.centerIn: parent
            spacing: 6
            Text {
                text: "⤓"
                color: dropHint.armed ? Theme.accent : Theme.muted
                font.pixelSize: 22
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text {
                text: root.title + " 을(를) 여기에 놓으면 다시 붙습니다"
                color: dropHint.armed ? Theme.accentDim : Theme.muted
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.horizontalCenter: parent.horizontalCenter
            }
            AppButton {
                text: "되돌리기"
                anchors.horizontalCenter: parent.horizontalCenter
                onClicked: root.dockBack()
            }
        }
    }

    // 드래그 고스트 — 마우스를 따라다니는 작은 라벨 창 (입력을 가로채지 않음)
    Window {
        id: ghost
        width: ghostLabel.implicitWidth + 28
        height: 34
        flags: Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool
             | Qt.WindowTransparentForInput | Qt.WindowDoesNotAcceptFocus
        color: "transparent"
        visible: false
        property bool overDock: false
        Rectangle {
            anchors.fill: parent
            radius: 6
            color: ghost.overDock ? Theme.mutedSoft : Theme.accentSoft
            border.width: 2
            border.color: ghost.overDock ? Theme.muted : Theme.accent
            Text {
                id: ghostLabel
                anchors.centerIn: parent
                text: ghost.overDock ? "제자리" : (root.title + " ⧉")
                color: ghost.overDock ? Theme.sub : Theme.accentDim
                font.pixelSize: 11
                font.bold: true
                font.family: Theme.fontFamily
            }
        }
    }

    Window {
        id: win
        width: root.floatWidth
        height: root.floatHeight
        minimumWidth: 320
        minimumHeight: 240
        title: root.title + " — Road Painter"
        color: Theme.bg
        flags: Qt.Window
        Item {
            id: floatHost
            anchors.fill: parent
            anchors.margins: 6
        }
        onClosing: function(close) {
            close.accepted = false
            root.dockBack()
        }
    }
}

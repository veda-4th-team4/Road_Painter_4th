import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Item {
    id: page
    focus: true

    // ── 배치(레이아웃) 상태 ──────────────────────────────────────────
    // 창을 떼어냈을 때뿐 아니라 분할선을 조금이라도 움직이면 "배치 초기화"가 뜬다.
    property bool layoutTouched: false
    property var defaultLayout: undefined
    // Top View 오른쪽 도구 팔레트를 접었는지. 접으면 그만큼 작도 캔버스가 넓어진다.
    property bool paletteCollapsed: false
    // 방향키를 누르고 있는 중인가 (창 비활성화 시 STOP 을 보내기 위한 상태)
    property bool dirKeyHeld: false
    readonly property bool layoutDirty: layoutTouched || paletteCollapsed
        || cctvPanel.floating || topPanel.floating || logPanel.floating
        || dashPanel.floating || manualPanel.floating

    // .13 PNM의 CH1~CH4 미리보기 화면인가.
    // 테스트 모드도 실제 CCTV 영상을 확인할 수 있어야 하므로 같은 채널 그리드를 쓴다.
    // 서버 명령만 생략하고 RTSP 선택·미리보기 흐름은 일반 모드와 동일하다.
    readonly property bool channelGridMode:
        Backend.workingChannel === 0

    function captureLayout() {
        defaultLayout = {
            outer:  outerSplit.saveState(),
            left:   leftSplit.saveState(),
            view:   viewSplit.saveState(),
            bottom: bottomSplit.saveState()
        }
    }
    function resetLayout() {
        cctvPanel.dockBack(); topPanel.dockBack()
        logPanel.dockBack();  dashPanel.dockBack()
        manualPanel.dockBack()
        paletteCollapsed = false
        if (defaultLayout !== undefined) {
            outerSplit.restoreState(defaultLayout.outer)
            leftSplit.restoreState(defaultLayout.left)
            viewSplit.restoreState(defaultLayout.view)
            bottomSplit.restoreState(defaultLayout.bottom)
        }
        layoutTouched = false
    }

    Component.onCompleted: {
        forceActiveFocus()
        // 첫 레이아웃이 끝난 뒤의 크기를 "기본 배치"로 기억해둔다
        Qt.callLater(captureLayout)
    }

    // 🔴 4채널에서는 [작업하기] **버튼**을 눌러 작업 화면으로 들어온다. Button 은
    //    클릭하면 포커스를 가져가므로, 그대로 두면 방향키가 여기 Keys 핸들러까지
    //    오지 않아 로봇 수동 조작이 먹지 않는다. 1채널 때는 로그인 직후 곧바로
    //    작업 화면이라(Component.onCompleted 의 forceActiveFocus) 안 드러났던 문제다.
    //    빈 곳을 클릭하면 아래 MouseArea 가 살려주긴 하지만, 그건 조작자가 원인을
    //    알아야 가능한 복구다 — 채널에 들어올 때마다 여기서 되돌린다.
    Connections {
        target: Backend
        function onChannelChanged() {
            if (Backend.workingChannel > 0) page.forceActiveFocus()
        }
    }

    Keys.onPressed: function(event) {
        if (event.isAutoRepeat) { event.accepted = true; return }
        // ⚠️ 그리드 화면에서는 **이동 명령을 보내지 않는다.** 아직 채널을 안 고른
        //    상태라 조작자가 로봇을 화면으로 보고 있지 않다. 안 보고 움직이는 건
        //    위험하다. (ESTOP 은 아래에서 화면과 무관하게 항상 받는다)
        if (page.channelGridMode) {
            switch (event.key) {
            case Qt.Key_Up: case Qt.Key_Down: case Qt.Key_Left: case Qt.Key_Right:
            case Qt.Key_PageUp: case Qt.Key_PageDown:
                event.accepted = true; return
            }
        }
        switch (event.key) {
        case Qt.Key_Up:    page.dirKeyHeld = true; Backend.sendRobotCmd("FORWARD", "전진"); event.accepted = true; break
        case Qt.Key_Down:  page.dirKeyHeld = true; Backend.sendRobotCmd("BACKWARD", "후진"); event.accepted = true; break
        case Qt.Key_Left:  page.dirKeyHeld = true; Backend.sendRobotCmd("TURN_LEFT", "좌회전"); event.accepted = true; break
        case Qt.Key_Right: page.dirKeyHeld = true; Backend.sendRobotCmd("TURN_RIGHT", "우회전"); event.accepted = true; break
        // ⚠️ 비상정지(Space)는 여기서 처리하지 않는다. 이 핸들러는 page 가
        //    activeFocus 를 가진 동안에만 오는데, 버튼을 한 번 클릭하면 포커스가
        //    그 버튼으로 넘어가 Space 가 **직전 버튼을 다시 누르는** 키가 된다
        //    (ESTOP 버튼이었다면 그대로 해제). 아래 Shortcut 이 포커스와 무관하게
        //    받고, 거는 방향으로만 동작한다.
        // 노즐은 누르고 있는 동작이 아니라 상태 전환이라 눌렀을 때 한 번만
        case Qt.Key_PageUp:   Backend.setNozzle(false); event.accepted = true; break
        case Qt.Key_PageDown: Backend.setNozzle(true);  event.accepted = true; break
        case Qt.Key_Escape:
        case Qt.Key_Return:
        case Qt.Key_Enter:
            // 더블클릭/우클릭과 동일하게 작도를 끝낸다
            if (Backend.drawing) { Backend.finishDrawing(); event.accepted = true }
            break
        }
    }
    Keys.onReleased: function(event) {
        if (event.isAutoRepeat) return
        if (event.key === Qt.Key_Up || event.key === Qt.Key_Down ||
            event.key === Qt.Key_Left || event.key === Qt.Key_Right) {
            page.dirKeyHeld = false
            Backend.sendRobotCmd("STOP", "정지")
            event.accepted = true
        }
    }
    // 방향키를 누른 채 Alt+Tab 하면 keyRelease 가 오지 않는다. 창이 비활성화되면
    // activeFocus 가 풀리므로, 그때 눌린 상태였다면 STOP 을 보낸다.
    onActiveFocusChanged: {
        if (!page.activeFocus && page.dirKeyHeld) {
            page.dirKeyHeld = false
            Backend.sendRobotCmd("STOP", "정지")
        }
    }
    MouseArea {
        anchors.fill: parent
        z: -1
        onClicked: page.forceActiveFocus()
    }
    // 🔴 비상정지 — 어떤 컨트롤이 포커스를 갖고 있어도 받는다.
    //    Shortcut 은 키 이벤트가 포커스 아이템(Button 등)에 배달되기 전에 처리되므로
    //    "직전에 클릭한 버튼이 Space 로 다시 눌리는" 경로 자체가 없어진다.
    //    해제(RESUME)는 절대 이 키로 하지 않는다 — 화면의 ESTOP 버튼으로만 푼다.
    Shortcut {
        sequence: "Space"
        context: Qt.WindowShortcut
        autoRepeat: false
        onActivated: Backend.engageEstop()
    }
    Shortcut { sequence: "Ctrl+Z"; enabled: Backend.drawing || Backend.hasPath; onActivated: Backend.undo() }
    Shortcut { sequence: "Ctrl+A"; enabled: Backend.hasPath; onActivated: topPane.view.selectAllActive() }
    Shortcut { sequence: "Ctrl+0"; onActivated: topPane.view.fitView() }
    Shortcut { sequences: [StandardKey.ZoomIn]; onActivated: topPane.view.zoomBy(1.25) }
    Shortcut { sequences: [StandardKey.ZoomOut]; onActivated: topPane.view.zoomBy(1 / 1.25) }
    Shortcut {
        sequence: StandardKey.Delete
        enabled: topPane.view.selectionCount > 0 && !Backend.jobActive
        onActivated: topPane.view.deleteSelection()
    }

    // 오른쪽 주 버튼 하나로 단계가 흐르도록:
    //   경로 작성 → 도면 전송(BLUEPRINT, 저장만) → 그림그리기 시작(START_DRAW) → 새 작업
    // ⚠️ START_DRAW 를 누르면 접근 → 도색 → 완료까지 서버가 자동으로 진행한다.
    //    중간에 누를 버튼은 없고, 끝은 DRAW_DONE 으로만 온다.
    function primaryText() {
        if (Backend.jobActive) return "작업 진행 중…"
        if (Backend.phase === "done") return "새 작업"
        if (Backend.canStart) return "그림그리기 시작"
        if (Backend.canCommit) return "도면 전송 (서버 저장)"
        return "경로 작성하기"
    }
    function primaryAction() {
        if (Backend.jobActive) return
        if (Backend.phase === "done") { Backend.clearMission(); return }
        if (Backend.canStart) { startConfirmPopup.open(); return }
        if (Backend.canCommit) { sendConfirmPopup.open(); return }
        if (!Backend.drawing && !Backend.hasPath) Backend.startDrawSession(false)
    }

    component HelpIcon: Item {
        id: helpIcon
        property string helpTitle: "도움말"
        property string helpText: ""
        property bool opensPopover: true
        signal activated()
        width: 20
        height: 20
        Accessible.role: Accessible.Button
        Accessible.name: helpTitle

        Rectangle {
            anchors.fill: parent
            radius: width / 2
            color: helpMouse.containsMouse || helpPopup.opened ? Theme.accentSoft : "transparent"
            border.width: 1
            border.color: helpMouse.containsMouse || helpPopup.opened ? Theme.accent : Theme.stroke
            Text {
                anchors.centerIn: parent
                text: "?"
                color: helpMouse.containsMouse || helpPopup.opened ? Theme.accentDim : Theme.muted
                font.pixelSize: 12
                font.bold: true
                font.family: Theme.fontFamily
            }
            MouseArea {
                id: helpMouse
                anchors.fill: parent
                hoverEnabled: true
                cursorShape: Qt.PointingHandCursor
                ToolTip.visible: containsMouse && !helpPopup.opened
                ToolTip.text: helpIcon.helpTitle
                onClicked: {
                    helpIcon.activated()
                    if (helpIcon.opensPopover) helpPopup.open()
                }
            }
        }

        Popup {
            id: helpPopup
            parent: Overlay.overlay
            width: Math.min(310, page.width - 24)
            padding: 14
            closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
            x: Math.max(12, Math.min(parent.width - width - 12,
                       helpIcon.mapToItem(parent, 0, 0).x + helpIcon.width - width))
            y: Math.max(12, Math.min(parent.height - height - 12,
                       helpIcon.mapToItem(parent, 0, 0).y + helpIcon.height + 6))
            background: Rectangle {
                radius: 6
                color: Theme.surface
                border.width: 1
                border.color: Theme.accent
            }
            contentItem: Column {
                width: helpPopup.width - 28
                spacing: 7
                Text {
                    width: parent.width
                    text: helpIcon.helpTitle
                    color: Theme.text
                    font.pixelSize: 13
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                Text {
                    width: parent.width
                    text: helpIcon.helpText
                    color: Theme.sub
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    lineHeight: 1.25
                    wrapMode: Text.WordWrap
                }
            }
        }
    }

    component Pill: Rectangle {
        id: pill
        property string label: ""
        property bool ok: false
        property bool bad: false
        radius: 14
        color: Theme.panel
        border.width: 1
        border.color: Theme.stroke
        implicitWidth: row.implicitWidth + 18
        implicitHeight: 28
        Row {
            id: row
            anchors.centerIn: parent
            spacing: 6
            Rectangle {
                width: 7; height: 7; radius: 4
                anchors.verticalCenter: parent.verticalCenter
                color: pill.bad ? Theme.danger
                     : (pill.ok ? Theme.success : Theme.muted)
            }
            Text {
                text: pill.label
                color: Theme.text
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
        }
    }

    // 헤더 계측값 — 라벨은 조용하게, 값만 눈에 띄게
    component Metric: Row {
        property string k: ""
        property string v: ""
        property bool warn: false
        spacing: 4
        Text {
            text: parent.k
            color: Theme.muted
            font.pixelSize: 10
            font.family: Theme.fontFamily
            anchors.verticalCenter: parent.verticalCenter
        }
        Text {
            text: parent.v
            color: parent.warn ? Theme.warn : Theme.text
            font.pixelSize: 12
            font.bold: true
            font.family: Theme.fontFamily
            anchors.verticalCenter: parent.verticalCenter
        }
    }

    component KvRow: Row {
        property string k: ""
        property string v: ""
        property color vColor: Theme.text
        width: parent ? parent.width : 200
        spacing: 8
        Text {
            width: parent.width * 0.42
            text: k
            color: Theme.muted
            font.pixelSize: 12
            font.family: Theme.fontFamily
        }
        Text {
            width: parent.width * 0.58
            text: v
            color: vColor
            font.pixelSize: 12
            font.bold: true
            font.family: Theme.fontFamily
            horizontalAlignment: Text.AlignRight
            elide: Text.ElideRight
        }
    }

    // ── 헤더 ────────────────────────────────────────────────────────
    Rectangle {
        id: header
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        height: 56
        color: Theme.surface
        z: 3
        Rectangle {
            anchors.bottom: parent.bottom
            width: parent.width
            height: 1
            color: Theme.stroke
        }

        Row {
            anchors.left: parent.left
            anchors.leftMargin: 18
            anchors.verticalCenter: parent.verticalCenter
            spacing: 10
            Column {
                spacing: 1
                Text {
                    text: "Road Painter"
                    color: Theme.text
                    font.pixelSize: 16
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                Text {
                    text: Backend.testMode ? "테스트 모드"
                        : (Backend.userId.length ? Backend.userId + " · 한화비전 관제" : "한화비전 · 관제")
                    color: Theme.muted
                    font.pixelSize: 10
                    font.family: Theme.fontFamily
                }
            }
            // 지금 어느 채널로 작업 중인지 + CH1~CH4 목록으로 돌아가기.
            Row {
                visible: Backend.workingChannel > 0
                anchors.verticalCenter: parent.verticalCenter
                spacing: 6
                AppButton {
                    text: "◀ 채널 목록"
                    enabled: !Backend.jobActive
                    ToolTip.visible: hovered && Backend.jobActive
                    // 작업 중 채널을 바꾸면 지금 로봇을 보고 있는 카메라가 바뀌어
                    // pose 공급이 끊긴다. 서버는 안 막으므로 여기서 막는다.
                    ToolTip.text: "작업 중에는 채널을 바꿀 수 없습니다 — 먼저 작업을 취소하세요"
                    onClicked: Backend.showChannelGrid()
                }
                Rectangle {
                    anchors.verticalCenter: parent.verticalCenter
                    radius: 4
                    color: Theme.accentSoft
                    border.color: Theme.accent
                    border.width: 1
                    width: chLabel.implicitWidth + 16
                    height: 24
                    Text {
                        id: chLabel
                        anchors.centerIn: parent
                        text: "CH" + Backend.workingChannel
                        color: Theme.accentDim
                        font.pixelSize: 11
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                }
            }
        }

        Row {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            spacing: 8
            Pill {
                label: "CCTV  " + (Backend.cctvOnline ? "연결됨" : "끊김")
                ok: Backend.cctvOnline
                bad: !Backend.cctvOnline
            }
            Pill {
                label: "서버  " + Backend.serverLabel
                ok: Backend.serverConnected || Backend.testMode
                bad: !Backend.serverConnected && !Backend.testMode && Backend.userId.length > 0
            }
            Pill {
                label: "로봇  " + (Backend.robotOnline ? "연결됨" : "오프라인")
                ok: Backend.robotOnline
                bad: !Backend.robotOnline || Backend.robotState === "ESTOPPED"
            }
        }

        Row {
            anchors.right: parent.right
            anchors.rightMargin: 16
            anchors.verticalCenter: parent.verticalCenter
            spacing: 10
            Metric {
                anchors.verticalCenter: parent.verticalCenter
                k: "FPS"
                v: Backend.cctvFps.toFixed(1)
                warn: Backend.cctvOnline && Backend.cctvFps < 10
            }
            Metric {
                anchors.verticalCenter: parent.verticalCenter
                k: "지연"
                v: Math.round(Backend.cctvLatencyMs) + " ms"
                warn: Backend.cctvLatencyMs > 300
            }
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: 1; height: 20; color: Theme.stroke
            }
            AppButton {
                visible: !Backend.testMode && !Backend.serverConnected
                text: "재연결"
                onClicked: Backend.reconnectServer()
            }
            AppButton {
                text: "배치 초기화"
                accent: true
                ToolTip.visible: hovered
                ToolTip.text: "분할 크기와 떼어낸 창을 모두 기본 배치로 되돌립니다"
                visible: page.layoutDirty
                onClicked: page.resetLayout()
            }
            AppButton {
                text: "작업 이력"
                enabled: !Backend.homographyPending
                ToolTip.visible: hovered
                ToolTip.text: "지금까지 그린 도면 — 계획→결과 확인, 다시 그리기, 수정, 이름변경, 삭제"
                onClicked: historyDialog.open()
                // 새 이력이 쌓이면 개수를 배지로 알려준다
                Rectangle {
                    visible: Backend.historyCount > 0
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.rightMargin: -4
                    anchors.topMargin: -4
                    width: 16; height: 14; radius: 7
                    color: Theme.accent
                    Text {
                        anchors.centerIn: parent
                        text: Math.min(Backend.historyCount, 99)
                        color: "#FFFFFF"
                        font.pixelSize: 9
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                }
            }
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "Road Painter 사용 순서"
                helpText: ""
                opensPopover: false
                onActivated: tutorialPopup.open()
            }
            AppButton {
                text: "설정"
                enabled: !Backend.homographyPending
                onClicked: settingsPopup.open()
            }
            AppButton {
                visible: page.channelGridMode
                enabled: !Backend.homographyPending
                text: "로그아웃"
                ToolTip.visible: hovered
                ToolTip.text: "현재 계정에서 로그아웃합니다"
                onClicked: logoutPopup.open()
            }
            AppButton {
                danger: Backend.robotState !== "ESTOPPED"
                accent: Backend.robotState === "ESTOPPED"
                text: Backend.robotState === "ESTOPPED" ? "ESTOP 해제" : "ESTOP"
                onClicked: Backend.toggleEstop()
            }
        }
    }

    // ── 통지 배너 (DRAW_FAIL · 캘리브레이션 없음 · 수동조작 차단 등) ──
    Rectangle {
        id: banner
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: header.bottom
        height: Backend.notice.length > 0 ? Math.max(38, bannerText.implicitHeight + 16) : 0
        visible: height > 0
        clip: true
        z: 2
        color: Backend.noticeLevel === "error" ? Theme.dangerSoft
             : (Backend.noticeLevel === "warn" ? Theme.warnSoft : Theme.mutedSoft)
        Rectangle {
            anchors.left: parent.left
            width: 3; height: parent.height
            color: Backend.noticeLevel === "error" ? Theme.danger
                 : (Backend.noticeLevel === "warn" ? Theme.warn : Theme.muted)
        }
        Behavior on height { NumberAnimation { duration: 120 } }

        Text {
            id: bannerText
            anchors.left: parent.left
            anchors.leftMargin: 16
            anchors.right: bannerBtns.left
            anchors.rightMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            text: Backend.notice
            color: Theme.text
            font.pixelSize: 12
            font.family: Theme.fontFamily
            wrapMode: Text.WordWrap
        }
        Row {
            id: bannerBtns
            anchors.right: parent.right
            anchors.rightMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            spacing: 6
            AppButton {
                visible: Backend.calibMissing
                text: "캘리브 설정 열기"
                onClicked: {
                    settingsPopup.tabIndex = 1
                    settingsPopup.open()
                }
            }
            AppButton { text: "닫기"; onClicked: Backend.dismissNotice() }
        }
    }

    // ── 4채널 미리보기 (PNM) ─────────────────────────────────────────
    // 로그인 후 CH1~CH4를 모두 보여주고, 작업 채널을 고르면 작업 화면으로 전환한다.
    // 아래 작업 화면(outerSplit)은 유지한 채 그리드만 덮는다.
    ChannelGrid {
        id: channelGridView
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: banner.bottom
        anchors.bottom: parent.bottom
        z: 2
        visible: page.channelGridMode
    }

    // ── 본문: 전부 드래그로 크기 조절 가능 ───────────────────────────
    SplitView {
        id: outerSplit
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: banner.bottom
        anchors.bottom: parent.bottom
        orientation: Qt.Horizontal
        // 그리드가 떠 있는 동안은 숨긴다. visible=false 면 입력도 안 받으므로
        // 뒤에서 실수로 작도가 되는 일이 없다. SplitView 의 분할 상태는 유지된다.
        visible: !page.channelGridMode

        handle: Rectangle {
            implicitWidth: 6
            implicitHeight: 6
            color: SplitHandle.pressed ? Theme.accent
                 : (SplitHandle.hovered ? Qt.lighter(Theme.accent, 1.5) : Theme.bg)
            // 분할선을 잡는 순간 "배치가 바뀌었다"고 표시 → 초기화 버튼이 뜬다
            property bool grabbed: SplitHandle.pressed
            onGrabbedChanged: if (grabbed) page.layoutTouched = true
            Rectangle {
                anchors.centerIn: parent
                width: 2; height: 26; radius: 1
                color: SplitHandle.pressed ? "#FFFFFF" : Theme.stroke
            }
        }

        SplitView {
            id: leftSplit
            SplitView.fillWidth: true
            SplitView.minimumWidth: 420
            orientation: Qt.Vertical

            handle: Rectangle {
                implicitWidth: 6
                implicitHeight: 6
                color: SplitHandle.pressed ? Theme.accent
                     : (SplitHandle.hovered ? Qt.lighter(Theme.accent, 1.5) : Theme.bg)
                property bool grabbed: SplitHandle.pressed
                onGrabbedChanged: if (grabbed) page.layoutTouched = true
                Rectangle {
                    anchors.centerIn: parent
                    width: 26; height: 2; radius: 1
                    color: SplitHandle.pressed ? "#FFFFFF" : Theme.stroke
                }
            }

            SplitView {
                id: viewSplit
                SplitView.fillHeight: true
                SplitView.minimumHeight: 240
                orientation: Qt.Horizontal

                handle: Rectangle {
                    implicitWidth: 6
                    implicitHeight: 6
                    color: SplitHandle.pressed ? Theme.accent
                         : (SplitHandle.hovered ? Qt.lighter(Theme.accent, 1.5) : Theme.bg)
                    property bool grabbed: SplitHandle.pressed
                    onGrabbedChanged: if (grabbed) page.layoutTouched = true
                    Rectangle {
                        anchors.centerIn: parent
                        width: 2; height: 26; radius: 1
                        color: SplitHandle.pressed ? "#FFFFFF" : Theme.stroke
                    }
                }

                // CCTV 자리 — 16:9라 조금 넓게 시작
                Item {
                    id: cctvSlot
                    SplitView.preferredWidth: viewSplit.width * 0.56
                    SplitView.minimumWidth: 220
                }
                // TopView 자리
                Item {
                    id: topSlot
                    SplitView.fillWidth: true
                    SplitView.minimumWidth: 300
                }
            }

            // 아래 띠는 가로로 나눠 쓴다 — 로그가 폭을 다 먹고 비어 있어서
            // 오른쪽 제어판에서 자리를 많이 차지하던 수동 주행 패드를 여기로 옮겼다.
            SplitView {
                id: bottomSplit
                SplitView.preferredHeight: 190
                SplitView.minimumHeight: 70
                orientation: Qt.Horizontal

                handle: Rectangle {
                    implicitWidth: 6
                    implicitHeight: 6
                    color: SplitHandle.pressed ? Theme.accent
                         : (SplitHandle.hovered ? Qt.lighter(Theme.accent, 1.5) : Theme.bg)
                    property bool grabbed: SplitHandle.pressed
                    onGrabbedChanged: if (grabbed) page.layoutTouched = true
                    Rectangle {
                        anchors.centerIn: parent
                        width: 2; height: 26; radius: 1
                        color: SplitHandle.pressed ? "#FFFFFF" : Theme.stroke
                    }
                }

                Item {
                    id: logSlot
                    SplitView.fillWidth: true
                    SplitView.minimumWidth: 260
                }
                Item {
                    id: manualSlot
                    SplitView.preferredWidth: 330
                    SplitView.minimumWidth: 240
                    SplitView.maximumWidth: 460
                }
            }
        }

        Item {
            id: dashSlot
            SplitView.preferredWidth: 300
            SplitView.minimumWidth: 250
            SplitView.maximumWidth: 480
        }
    }

    // ── 도킹 패널들 (헤더를 끌면 창으로 분리됨) ─────────────────────
    DockPanel {
        id: cctvPanel
        dock: cctvSlot
        title: "CCTV 실시간 영상"
        subtitle: (Backend.camIp.length ? Backend.camIp : "")
                  + (Backend.arucoSummary.length
                     ? ((Backend.camIp.length ? "  ·  " : "") + Backend.arucoSummary) : "")
        floatWidth: 960
        floatHeight: 620

        headerExtra: [
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: arucoLabel.implicitWidth + 22
                height: 20
                radius: 10
                color: Backend.arucoOverlay ? Theme.accentSoft : Theme.mutedSoft
                border.width: 1
                border.color: Backend.arucoOverlay ? Theme.accent : Theme.stroke
                Row {
                    anchors.centerIn: parent
                    spacing: 5
                    Rectangle {
                        width: 6; height: 6; radius: 3
                        anchors.verticalCenter: parent.verticalCenter
                        color: Backend.arucoOverlay ? Theme.accent : Theme.muted
                    }
                    Text {
                        id: arucoLabel
                        text: "ArUco " + (Backend.arucoOverlay ? "ON" : "OFF")
                        color: Backend.arucoOverlay ? Theme.accentDim : Theme.muted
                        font.pixelSize: 10
                        font.bold: true
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    hoverEnabled: true
                    ToolTip.visible: containsMouse
                    ToolTip.text: "OpenCV 마커 검출과 화면 표시를 함께 켜고 끕니다\n" +
                                  (Backend.arucoSummary.length ? Backend.arucoSummary : "검출된 마커 없음")
                    onClicked: Backend.arucoOverlay = !Backend.arucoOverlay
                }
            },
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "CCTV 영상"
                helpText: "선택한 채널의 RTSP 영상을 확인합니다. ArUco를 끄면 마커 검출과 외곽선 표시가 모두 중지됩니다. 연결 실패 시 설정의 카메라 IP와 스트림 주소를 확인하세요."
            }
        ]

        VideoPane {
            anchors.fill: parent
            anchors.margins: 8
            title: "Camera RTSP"
            topRole: false
            highlight: false
        }
    }

    DockPanel {
        id: topPanel
        dock: topSlot
        title: "Top View"
        floatWidth: 780
        floatHeight: 900

        headerExtra: [
            // 확대/축소 — 휠로도 되지만 버튼도 둔다
            Row {
                anchors.verticalCenter: parent.verticalCenter
                spacing: 2
                component ZoomBtn: Rectangle {
                    property alias label: zt.text
                    signal act()
                    width: 22; height: 20; radius: 4
                    color: zm.containsMouse ? Theme.elevated : "transparent"
                    border.width: 1
                    border.color: Theme.stroke
                    Text {
                        id: zt
                        anchors.centerIn: parent
                        color: Theme.sub
                        font.pixelSize: 11
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                    MouseArea {
                        id: zm
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        onClicked: parent.act()
                    }
                }
                ZoomBtn { label: "−"; onAct: topPane.view.zoomBy(1 / 1.25) }
                // 배율 칩 — 클릭하면 그 자리에서 숫자를 직접 입력한다
                Rectangle {
                    id: zoomChip
                    property bool editing: false
                    width: 52
                    height: 20
                    radius: 4
                    color: editing ? Theme.surface : "transparent"
                    border.width: 1
                    border.color: editing ? Theme.accent
                                          : (zoomHover.hovered ? Theme.accent : Theme.stroke)

                    function beginEdit() {
                        zoomField.text = String(topPane.view.zoomPercent)
                        editing = true
                        zoomField.forceActiveFocus()
                        zoomField.selectAll()
                    }
                    function commit() {
                        const v = parseInt(zoomField.text)
                        if (isFinite(v) && v > 0) topPane.view.setZoomPercent(v)
                        editing = false
                    }

                    Text {
                        anchors.centerIn: parent
                        visible: !zoomChip.editing
                        text: topPane.view.zoomPercent + "%"
                        color: Theme.sub
                        font.pixelSize: 10
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                    TextInput {
                        id: zoomField
                        anchors.fill: parent
                        anchors.leftMargin: 4
                        anchors.rightMargin: 4
                        visible: zoomChip.editing
                        color: Theme.text
                        font.pixelSize: 10
                        font.bold: true
                        font.family: Theme.fontFamily
                        horizontalAlignment: TextInput.AlignHCenter
                        verticalAlignment: TextInput.AlignVCenter
                        selectByMouse: true
                        maximumLength: 4
                        validator: IntValidator { bottom: 25; top: 1200 }
                        inputMethodHints: Qt.ImhDigitsOnly
                        onAccepted: zoomChip.commit()
                        onActiveFocusChanged: if (!activeFocus && zoomChip.editing) zoomChip.commit()
                        Keys.onEscapePressed: zoomChip.editing = false
                    }
                    HoverHandler { id: zoomHover }
                    MouseArea {
                        anchors.fill: parent
                        visible: !zoomChip.editing
                        cursorShape: Qt.IBeamCursor
                        ToolTip.visible: zoomHover.hovered && !zoomChip.editing
                        ToolTip.text: "클릭 → 배율 직접 입력 (25~1200%)\n"
                                    + "휠 = 커서 기준 확대/축소\n휠클릭·Alt+드래그 = 화면 이동\n"
                                    + "더블클릭 = 전체 보기"
                        onClicked: zoomChip.beginEdit()
                        onDoubleClicked: topPane.view.fitView()
                    }
                }
                ZoomBtn { label: "+"; onAct: topPane.view.zoomBy(1.25) }
            },
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: scaleLabel.implicitWidth + 16
                height: 20
                radius: 10
                color: Theme.accentSoft
                border.width: 1
                border.color: Theme.accent
                Text {
                    id: scaleLabel
                    anchors.centerIn: parent
                    text: Backend.scaleText
                    color: Theme.accentDim
                    font.pixelSize: 10
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                HoverHandler { id: scaleHover }
                ToolTip.visible: scaleHover.hovered
                ToolTip.text: "TopView 1픽셀이 실제로 몇 mm인지 (현재 보정 기준)\n" + Backend.calibStatus
            },
            // 렌즈 보정 켜고 끄기. 배경만 바뀌므로 눌러가며 어느 쪽이 잘 펴지는지 바로 비교된다.
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: lensLabel.implicitWidth + 18
                height: 20
                radius: 10
                color: Backend.lensCorrection ? Theme.accentSoft : "transparent"
                border.width: 1
                border.color: Backend.lensCorrection ? Theme.accent : Theme.stroke
                opacity: Backend.lensReady ? 1.0 : 0.45
                Text {
                    id: lensLabel
                    anchors.centerIn: parent
                    text: "렌즈보정 " + (Backend.lensCorrection ? "ON" : "OFF")
                    color: Backend.lensCorrection ? Theme.accentDim : Theme.muted
                    font.pixelSize: 10
                    font.bold: Backend.lensCorrection
                    font.family: Theme.fontFamily
                }
                MouseArea {
                    id: lensMouse
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: Backend.lensCorrection = !Backend.lensCorrection
                }
                ToolTip.visible: lensMouse.containsMouse
                ToolTip.text: Backend.lensReady
                    ? ("TopView 배경의 렌즈 왜곡을 폅니다 (원본 영상·좌표는 그대로).\n"
                       + "호모그래피만으로는 가운데는 맞아도 끝쪽이 휩니다.\n" + Backend.lensSummary)
                    : "K·왜곡계수가 없습니다. 설정 → 캘리브에서 CCTV 캘리브레이션 JSON을 넣으세요."
            },
            // 로봇 아이콘 켜고 끄기. 265mm 기체를 실축으로 그리면 900×600 도면의 1/4 을
            // 덮어서, 로봇이 도면 한가운데 서 있으면 그리던 선이 통째로 가려진다.
            // 표시만 끈다 — 위치 수신·진행률은 계속 돈다.
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: robotVisLabel.implicitWidth + 18
                height: 20
                radius: 10
                color: Backend.robotVisible ? Theme.accentSoft : "transparent"
                border.width: 1
                border.color: Backend.robotVisible ? Theme.accent : Theme.stroke
                Text {
                    id: robotVisLabel
                    anchors.centerIn: parent
                    text: "로봇표시 " + (Backend.robotVisible ? "ON" : "OFF")
                    color: Backend.robotVisible ? Theme.accentDim : Theme.muted
                    font.pixelSize: 10
                    font.bold: Backend.robotVisible
                    font.family: Theme.fontFamily
                }
                MouseArea {
                    id: robotVisMouse
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: Backend.setRobotVisible(!Backend.robotVisible)
                }
                ToolTip.visible: robotVisMouse.containsMouse
                ToolTip.text: "TopView 위 로봇 아이콘을 숨깁니다.\n"
                    + "작도할 때 로봇이 도면을 가리면 끄세요.\n"
                    + "표시만 꺼지고 위치 수신·진행률 계산은 그대로 돕니다."
            },
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "Top View와 경로 작성"
                helpText: "프리셋을 캔버스로 끌거나 [경로 추가]로 점을 찍습니다. 기본 크기 조절은 비율을 유지하고, Shift를 누른 동안만 자유 변형합니다. 도색 반지름 200 mm 미만의 곡선은 빨간색으로 표시되며 전송할 수 없습니다."
            }
            // 곡선은 도면 기하가 원호 허용오차를 만족할 때 ARC로 보낸다.
            // 서버 v2가 실행 반지름과 펜 오프셋을 로봇 op으로 변환한다.
        ]

        Item {
            anchors.fill: parent
            anchors.margins: 8

            Text {
                id: tvHint
                anchors.left: parent.left
                anchors.right: parent.right
                height: 16
                text: Backend.phaseHint
                color: Theme.muted
                font.pixelSize: 11
                font.family: Theme.fontFamily
                elide: Text.ElideRight
                verticalAlignment: Text.AlignVCenter
            }

            // 월드 좌표(mm) 직접 입력 — 클릭 작도와 같은 경로에 쌓임
            Rectangle {
                id: mmBar
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: tvHint.bottom
                anchors.topMargin: 6
                height: 36
                radius: 6
                color: Theme.panel
                border.color: Theme.stroke
                border.width: 1
                clip: true

                function addPoint() {
                    const x = Number(mmX.text), y = Number(mmY.text)
                    if (mmX.text.length === 0 || mmY.text.length === 0) return
                    if (!isFinite(x) || !isFinite(y)) return
                    Backend.addWorldPointMm(x, y)
                }
                function addRect() {
                    const w = Number(mmW.text), h = Number(mmH.text)
                    if (!isFinite(w) || !isFinite(h) || w < 1 || h < 1) return
                    Backend.addRectWorldMm(w, h)
                }

                Row {
                    anchors.fill: parent
                    anchors.margins: 4
                    spacing: 4
                    component MmField: TextField {
                        width: 54
                        height: 28
                        color: Theme.text
                        leftPadding: 6
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        selectByMouse: true
                        background: Rectangle {
                            radius: 4
                            color: Theme.surface
                            border.width: 1
                            border.color: parent.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    component Unit: Text {
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
                    }
                    Unit { text: "점  X" }
                    MmField { id: mmX; placeholderText: "0"; placeholderTextColor: Theme.muted; onAccepted: mmBar.addPoint() }
                    Unit { text: "Y" }
                    MmField { id: mmY; placeholderText: "0"; placeholderTextColor: Theme.muted; onAccepted: mmBar.addPoint() }
                    Unit { text: "mm" }
                    AppButton {
                        width: 34; height: 28; text: "+"
                        enabled: !Backend.jobActive
                        ToolTip.visible: hovered
                        ToolTip.text: "월드 좌표(mm)로 점 추가 · Enter"
                        onClicked: mmBar.addPoint()
                    }
                    Rectangle { width: 1; height: 20; color: Theme.stroke; anchors.verticalCenter: parent.verticalCenter }
                    Unit { text: "사각" }
                    MmField { id: mmW; placeholderText: "W"; placeholderTextColor: Theme.muted; onAccepted: mmBar.addRect() }
                    Unit { text: "×" }
                    MmField { id: mmH; placeholderText: "H"; placeholderTextColor: Theme.muted; onAccepted: mmBar.addRect() }
                    Unit { text: "mm" }
                    AppButton {
                        width: 44; height: 28; text: "넣기"
                        enabled: !Backend.jobActive
                        ToolTip.visible: hovered
                        ToolTip.text: "캔버스 중앙에 W×H mm 사각형 · Enter"
                        onClicked: mmBar.addRect()
                    }
                }
            }

            VideoPane {
                id: topPane
                anchors.left: parent.left
                anchors.top: mmBar.bottom
                anchors.topMargin: 8
                anchors.bottom: parent.bottom
                anchors.right: toolStrip.left
                anchors.rightMargin: 20      // 접기 손잡이(14px)가 들어갈 자리
                title: "Top View"
                topRole: true
                highlight: Backend.drawing || Backend.jobActive
                          || Backend.phase === "ready"
            }

            // 도구 팔레트 접기 손잡이. 팔레트와 캔버스 사이 틈에 얹혀 있어
            // 접든 펼치든 그림을 가리지 않는다.
            Rectangle {
                id: paletteTab
                z: 5
                width: 14
                height: 62
                radius: 5
                anchors.right: toolStrip.left
                anchors.rightMargin: 2
                anchors.verticalCenter: toolStrip.verticalCenter
                color: paletteTabMa.containsMouse ? Theme.accentSoft : Theme.panel
                border.width: 1
                border.color: paletteTabMa.containsMouse ? Theme.accent : Theme.stroke

                Text {
                    anchors.centerIn: parent
                    text: page.paletteCollapsed ? "‹" : "›"
                    font.pixelSize: 16
                    font.bold: true
                    color: paletteTabMa.containsMouse ? Theme.accent : Theme.muted
                }
                MouseArea {
                    id: paletteTabMa
                    anchors.fill: parent
                    anchors.margins: -6        // 얇으니 클릭 영역만 넉넉히
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: page.paletteCollapsed = !page.paletteCollapsed
                    ToolTip.visible: containsMouse
                    ToolTip.delay: 400
                    ToolTip.text: page.paletteCollapsed ? "도구 팔레트 펼치기" : "도구 팔레트 접기"
                }
            }

            Rectangle {
                id: toolStrip
                anchors.right: parent.right
                anchors.top: mmBar.bottom
                anchors.topMargin: 8
                anchors.bottom: parent.bottom
                // 접으면 폭 0 → topPane 이 anchors 로 따라붙어 캔버스가 넓어진다
                width: page.paletteCollapsed ? 0 : 152
                visible: !page.paletteCollapsed
                Behavior on width { NumberAnimation { duration: 130; easing.type: Easing.OutCubic } }
                radius: 6
                color: Theme.panel
                border.color: Theme.stroke
                border.width: 1

                Flickable {
                    anchors.fill: parent
                    anchors.margins: 8
                    contentHeight: tsCol.implicitHeight
                    clip: true
                    boundsBehavior: Flickable.StopAtBounds

                    Column {
                        id: tsCol
                        // 도형 팔레트에서 지금 보고 있는 분류 (0 기본 · 1 노면 표시 · 2 글자)
                        property int paletteTab: 0
                        width: parent.width
                        spacing: 6

                        component SectionLabel: Text {
                            width: parent ? parent.width : 100
                            color: Theme.muted
                            font.pixelSize: 10
                            font.bold: true
                            font.family: Theme.fontFamily
                            topPadding: 4
                        }

                        Row {
                            width: parent.width
                            spacing: 6
                            AppButton {
                                width: parent.width - 26
                                accent: Backend.drawing
                                text: Backend.drawing ? "작도 끝내기" : "경로 추가"
                                enabled: !Backend.jobActive
                                onClicked: {
                                    if (Backend.drawing) Backend.finishDrawing()
                                    else Backend.startDrawSession(false)
                                }
                            }
                            HelpIcon {
                                anchors.verticalCenter: parent.verticalCenter
                                helpTitle: "경로 작성"
                                helpText: "프리셋은 캔버스로 끌어놓습니다. 직접 그릴 때는 점을 차례로 찍고 첫 점을 다시 누르면 닫힙니다. 더블클릭, 우클릭 또는 Esc로 열린 경로를 끝냅니다. 빈 곳을 드래그하면 여러 점을 선택합니다."
                            }
                        }
                        Text {
                            width: parent.width
                            wrapMode: Text.WordWrap
                            lineHeight: 1.15
                            visible: Backend.drawing
                            text: "점을 추가하는 중 · 첫 점 클릭으로 닫기"
                            color: Theme.muted
                            font.pixelSize: 10
                            font.family: Theme.fontFamily
                        }

                        // 도형 팔레트 — 세그먼트 탭으로 한 분류씩만 보여준다.
                        // 예전에는 세 분류(기본·노면·글자)를 세로로 전부 늘어놓아서 패널이
                        // 길어지고 스크롤해야 닿는 타일이 있었다. 탭이면 높이가 항상 일정해서
                        // 캔버스를 가리지도, 누를 때마다 레이아웃이 튀지도 않는다.
                        // 클릭 수는 그대로다 — 분류 전환 1회 + 타일 1회.
                        Rectangle {
                            width: parent.width
                            height: 28
                            radius: 6
                            color: Theme.surface
                            border.width: 1
                            border.color: Theme.stroke
                            Row {
                                anchors.fill: parent
                                anchors.margins: 2
                                spacing: 2
                                Repeater {
                                    model: ["기본", "노면 표시", "글자"]
                                    Rectangle {
                                        width: (parent.width - 4) / 3
                                        height: parent.height
                                        radius: 4
                                        color: tsCol.paletteTab === index ? Theme.accent : "transparent"
                                        Text {
                                            anchors.centerIn: parent
                                            text: modelData
                                            color: tsCol.paletteTab === index ? "#ffffff" : Theme.sub
                                            font.pixelSize: 11
                                            font.bold: tsCol.paletteTab === index
                                            font.family: Theme.fontFamily
                                        }
                                        MouseArea {
                                            anchors.fill: parent
                                            cursorShape: Qt.PointingHandCursor
                                            onClicked: tsCol.paletteTab = index
                                        }
                                    }
                                }
                            }
                        }

                        Grid {
                            visible: tsCol.paletteTab === 0
                            columns: 2
                            spacing: 4
                            width: parent.width
                            PresetTile { shape: "RECT"; label: "사각"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "TRIANGLE"; label: "삼각"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "CIRCLE"; label: "루프"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "LINE"; label: "직선"; rootItem: page; width: 64; height: 56 }
                        }

                        Grid {
                            visible: tsCol.paletteTab === 1
                            columns: 2
                            spacing: 4
                            width: parent.width
                            PresetTile { shape: "CROSSWALK"; label: "횡단보도"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "STOPLINE"; label: "정지선"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "ARROW_F"; label: "직진"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "ARROW_L"; label: "좌회전"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "ARROW_R"; label: "우회전"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "PARKING"; label: "주차구획"; rootItem: page; width: 64; height: 56 }
                            PresetTile { shape: "ZIGZAG"; label: "유도선"; rootItem: page; width: 64; height: 56 }
                        }

                        TextField {
                            id: textInput
                            visible: tsCol.paletteTab === 2
                            width: parent.width
                            height: 28
                            color: Theme.text
                            leftPadding: 6
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            selectByMouse: true
                            placeholderText: "예: 정지, SLOW"
                            placeholderTextColor: Theme.muted
                            onAccepted: tsCol.putText()
                            background: Rectangle {
                                radius: 4
                                color: Theme.surface
                                border.width: 1
                                border.color: textInput.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                        Row {
                            visible: tsCol.paletteTab === 2
                            width: parent.width
                            spacing: 4
                            TextField {
                                id: textHeight
                                width: 52; height: 28
                                text: "300"
                                color: Theme.text
                                leftPadding: 6
                                font.pixelSize: 11
                                font.family: Theme.fontFamily
                                inputMethodHints: Qt.ImhDigitsOnly
                                selectByMouse: true
                                onAccepted: tsCol.putText()
                                background: Rectangle {
                                    radius: 4
                                    color: Theme.surface
                                    border.width: 1
                                    border.color: textHeight.activeFocus ? Theme.accent : Theme.stroke
                                }
                            }
                            Text {
                                text: "mm 완성"
                                color: Theme.muted
                                font.pixelSize: 10
                                font.family: Theme.fontFamily
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            AppButton {
                                width: 58; height: 28
                                text: "넣기"
                                enabled: !Backend.jobActive
                                ToolTip.visible: hovered
                                ToolTip.text: "붓이 지나갈 획(중심선)으로 넣습니다.\n" +
                                              "입력값은 완성 도색 높이입니다 — 붓 폭까지 포함한,\n" +
                                              "실제로 칠해지는 세로 크기.\n" +
                                              "붓 폭(" + Backend.strokeWidthMm.toFixed(0) +
                                              "mm)의 3.5배 이상 권장"
                                onClicked: tsCol.putText()
                            }
                        }

                        Rectangle { width: parent.width; height: 1; color: Theme.stroke }

                        // ── 변환 ──────────────────────────────────
                        SectionLabel {
                            text: topPane.view.selectionCount > 0
                                  ? ("변환 · 선택 " + topPane.view.selectionCount + "점")
                                  : "변환 · 도형 전체"
                        }
                        Row {
                            width: parent.width
                            spacing: 4
                            component TBtn: AppButton {
                                width: 32; height: 28
                                enabled: topPane.view.hasActiveShape
                                         && !Backend.jobActive
                            }
                            TBtn {
                                text: "⟲"
                                ToolTip.visible: hovered; ToolTip.text: "좌로 90° 회전"
                                onClicked: Backend.rotateShape(90)
                            }
                            TBtn {
                                text: "⟳"
                                ToolTip.visible: hovered; ToolTip.text: "우로 90° 회전"
                                onClicked: Backend.rotateShape(-90)
                            }
                            TBtn {
                                text: "↔"
                                ToolTip.visible: hovered; ToolTip.text: "좌우 반전"
                                onClicked: Backend.flipShape(true)
                            }
                            TBtn {
                                text: "↕"
                                ToolTip.visible: hovered; ToolTip.text: "상하 반전"
                                onClicked: Backend.flipShape(false)
                            }
                        }
                        Row {
                            width: parent.width
                            spacing: 4
                            TextField {
                                id: rotDeg
                                width: 44; height: 26
                                text: "15"
                                color: Theme.text
                                leftPadding: 5
                                font.pixelSize: 11
                                font.family: Theme.fontFamily
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                selectByMouse: true
                                background: Rectangle {
                                    radius: 4; color: Theme.surface
                                    border.width: 1
                                    border.color: rotDeg.activeFocus ? Theme.accent : Theme.stroke
                                }
                            }
                            Text {
                                text: "°"
                                color: Theme.muted
                                font.pixelSize: 11
                                anchors.verticalCenter: parent.verticalCenter
                            }
                            AppButton {
                                width: 30; height: 26; text: "⟲"
                                enabled: topPane.view.hasActiveShape
                                onClicked: Backend.rotateShape(Number(rotDeg.text) || 0)
                            }
                            AppButton {
                                width: 30; height: 26; text: "⟳"
                                enabled: topPane.view.hasActiveShape
                                onClicked: Backend.rotateShape(-(Number(rotDeg.text) || 0))
                            }
                        }
                        Row {
                            width: parent.width
                            spacing: 4
                            AppButton {
                                width: 54; height: 26; text: "축소"
                                enabled: topPane.view.hasActiveShape
                                onClicked: Backend.scaleShape(1 / 1.1)
                            }
                            AppButton {
                                width: 54; height: 26; text: "확대"
                                enabled: topPane.view.hasActiveShape
                                onClicked: Backend.scaleShape(1.1)
                            }
                        }
                        AppButton {
                            width: parent.width
                            height: 26
                            text: "직선 정리"
                            enabled: Backend.hasPath && !Backend.jobActive
                            ToolTip.visible: hovered
                            ToolTip.text: "거의 일직선인 점을 합쳐 '직진 + 회전'만 남깁니다"
                            onClicked: Backend.simplifyPaths(Backend.strokeWidthMm * 0.2)
                        }

                        Rectangle { width: parent.width; height: 1; color: Theme.stroke }

                        AppButton {
                            width: parent.width
                            text: "마지막 점 취소"
                            enabled: Backend.hasPath && !Backend.jobActive
                            ToolTip.visible: hovered
                            ToolTip.text: "Ctrl+Z"
                            onClicked: Backend.undo()
                        }
                        AppButton {
                            width: parent.width
                            text: "전체 지우기"
                            // 편집 중인 점이 없어도, 완성해둔 도형이나 전송한 경로가 있으면 지울 수 있어야 한다
                            enabled: (Backend.hasPath || Backend.drawing || Backend.canEditMission)
                                     && !Backend.jobActive
                            ToolTip.visible: hovered
                            ToolTip.text: "작도한 도형을 모두 지웁니다"
                            onClicked: {
                                Backend.cancelDrawing()
                                if (Backend.canEditMission) Backend.clearMission()
                            }
                        }

                        function putText() {
                            const h = Number(textHeight.text)
                            // 하한 판정은 C++ 이 한다 — 붓 폭 기준이라 여기서 막으면
                            // 조작자가 사유를 못 본다 (변 길이 입력창과 같은 방식).
                            if (!textInput.text.length || !isFinite(h) || h <= 0) return
                            Backend.addTextWorldMm(textInput.text, h, false)
                        }
                    }
                }
            }
        }
    }

    // ── 수동 조작 (로그 옆) ──────────────────────────────────────────
    // 오른쪽 제어판이 세로로 길어져서, 넓게 비어 있던 아래 띠로 옮겼다.
    DockPanel {
        id: manualPanel
        dock: manualSlot
        title: "수동 조작"
        subtitle: "↑↓←→ · Space=ESTOP"
        floatWidth: 420
        floatHeight: 300

        headerExtra: [
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "수동 조작"
                helpText: "방향 버튼을 누르는 동안만 주행하고 떼면 정지합니다. 노즐은 올림/내림 상태를 전환합니다. 자동 경로 실행 중에는 수동 주행이 잠기며, Space는 언제든 비상 정지입니다."
            },
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                visible: !Backend.manualEnabled
                width: lockText.implicitWidth + 12
                height: 18
                radius: 9
                color: Theme.warnSoft
                border.width: 1
                border.color: Theme.warn
                Text {
                    id: lockText
                    anchors.centerIn: parent
                    text: "경로 실행 중 잠김"
                    color: Theme.warn
                    font.pixelSize: 9
                    font.bold: true
                    font.family: Theme.fontFamily
                }
            }
        ]

        Item {
            anchors.fill: parent
            anchors.margins: 10

            // 주행 패드 (왼쪽) + 노즐 (오른쪽). 아래 안내문과 겹치지 않게 살짝 위로.
            Row {
                anchors.centerIn: parent
                anchors.verticalCenterOffset: -7
                spacing: 16

                Grid {
                    columns: 3
                    spacing: 5
                    anchors.verticalCenter: parent.verticalCenter
                    opacity: Backend.manualEnabled ? 1.0 : 0.45
                    Item { width: 48; height: 30 }
                    AppButton {
                        width: 48; height: 30; text: "↑"
                        enabled: Backend.manualEnabled && Backend.robotState !== "ESTOPPED"
                        onPressed: Backend.sendRobotCmd("FORWARD", "전진")
                        onReleased: Backend.sendRobotCmd("STOP", "정지")
                        // 버튼 밖에서 떼거나(그랩 취소) 창이 비활성화되면 released 가
                        // 오지 않는다 — canceled 에도 STOP 을 보내야 로봇이 계속 달리지 않는다.
                        onCanceled: Backend.sendRobotCmd("STOP", "정지")
                    }
                    Item { width: 48; height: 30 }
                    AppButton {
                        width: 48; height: 30; text: "←"
                        enabled: Backend.manualEnabled && Backend.robotState !== "ESTOPPED"
                        onPressed: Backend.sendRobotCmd("TURN_LEFT", "좌회전")
                        onReleased: Backend.sendRobotCmd("STOP", "정지")
                        // 버튼 밖에서 떼거나(그랩 취소) 창이 비활성화되면 released 가
                        // 오지 않는다 — canceled 에도 STOP 을 보내야 로봇이 계속 달리지 않는다.
                        onCanceled: Backend.sendRobotCmd("STOP", "정지")
                    }
                    AppButton {
                        width: 48; height: 30; text: "■"
                        enabled: Backend.manualEnabled
                        onClicked: Backend.sendRobotCmd("STOP", "정지")
                    }
                    AppButton {
                        width: 48; height: 30; text: "→"
                        enabled: Backend.manualEnabled && Backend.robotState !== "ESTOPPED"
                        onPressed: Backend.sendRobotCmd("TURN_RIGHT", "우회전")
                        onReleased: Backend.sendRobotCmd("STOP", "정지")
                        // 버튼 밖에서 떼거나(그랩 취소) 창이 비활성화되면 released 가
                        // 오지 않는다 — canceled 에도 STOP 을 보내야 로봇이 계속 달리지 않는다.
                        onCanceled: Backend.sendRobotCmd("STOP", "정지")
                    }
                    Item { width: 48; height: 30 }
                    AppButton {
                        width: 48; height: 30; text: "↓"
                        enabled: Backend.manualEnabled && Backend.robotState !== "ESTOPPED"
                        onPressed: Backend.sendRobotCmd("BACKWARD", "후진")
                        onReleased: Backend.sendRobotCmd("STOP", "정지")
                        // 버튼 밖에서 떼거나(그랩 취소) 창이 비활성화되면 released 가
                        // 오지 않는다 — canceled 에도 STOP 을 보내야 로봇이 계속 달리지 않는다.
                        onCanceled: Backend.sendRobotCmd("STOP", "정지")
                    }
                    Item { width: 48; height: 30 }
                }

                Rectangle { width: 1; height: 96; color: Theme.stroke
                            anchors.verticalCenter: parent.verticalCenter }

                // 노즐 올림/내림 — 페이지 단축키 PgUp / PgDn 과 같은 동작
                Column {
                    spacing: 5
                    anchors.verticalCenter: parent.verticalCenter
                    Text {
                        text: "노즐 · " + (Backend.nozzleDown ? "내려짐" : "올려짐")
                        color: Backend.nozzleDown ? Theme.success : Theme.sub
                        font.pixelSize: 11
                        font.bold: true
                        font.family: Theme.fontFamily
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    AppButton {
                        width: 78; height: 30
                        text: "▲ 올림"
                        accent: !Backend.nozzleDown
                        enabled: Backend.manualEnabled
                        ToolTip.visible: hovered
                        ToolTip.text: "노즐을 들어 올립니다 (PageUp)"
                        onClicked: Backend.setNozzle(false)
                    }
                    AppButton {
                        width: 78; height: 30
                        text: "▼ 내림"
                        accent: Backend.nozzleDown
                        enabled: Backend.manualEnabled
                        ToolTip.visible: hovered
                        ToolTip.text: "노즐을 내려 도장 위치로 둡니다 (PageDown)"
                        onClicked: Backend.setNozzle(true)
                    }
                }
            }

            Text {
                anchors.bottom: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                width: parent.width
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
                text: Backend.manualEnabled
                    ? "↑↓←→ 주행 · PgUp/PgDn 노즐 · Space = ESTOP"
                    : "자동 경로 실행 중에는 수동 조작이 차단됩니다."
                color: Theme.muted
                font.pixelSize: 10
                font.family: Theme.fontFamily
            }
        }
    }

    DockPanel {
        id: logPanel
        dock: logSlot
        title: "시스템 로그"
        subtitle: "최신 항목으로 자동 스크롤"
        floatWidth: 720
        floatHeight: 420

        headerExtra: [
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "시스템 로그"
                helpText: "서버 연결, 경로 전송, 작업 응답과 오류를 시간순으로 표시합니다. 문제를 확인할 때 지우기 전에 마지막 오류 문구를 먼저 확인하세요."
            },
            AppButton {
                anchors.verticalCenter: parent.verticalCenter
                height: 22
                text: "지우기"
                onClicked: Backend.clearRobotLog()
            }
        ]

        ScrollView {
            id: logScroll
            anchors.fill: parent
            anchors.margins: 8
            clip: true
            // 위로 올려 과거 로그를 보는 중이면 방해하지 않는다
            function nearBottom() {
                const sb = ScrollBar.vertical
                if (!sb || sb.size >= 1) return true
                return sb.position >= 1 - sb.size - 0.05
            }
            function toBottom() {
                const sb = ScrollBar.vertical
                if (sb && sb.size < 1) sb.position = 1 - sb.size
            }
            TextArea {
                id: logArea
                readOnly: true
                text: Backend.robotLog.length ? Backend.robotLog : "[—] 로그 없음"
                color: Theme.sub
                font.family: Theme.fontFamily
                font.pixelSize: 11
                wrapMode: TextArea.Wrap
                background: null
                selectByMouse: true
                property bool stick: true
                onTextChanged: {
                    stick = logScroll.nearBottom()
                    Qt.callLater(function() { if (logArea.stick) logScroll.toBottom() })
                }
            }
        }
    }

    DockPanel {
        id: dashPanel
        dock: dashSlot
        title: "제어판"
        floatWidth: 360
        floatHeight: 820

        headerExtra: [
            HelpIcon {
                anchors.verticalCenter: parent.verticalCenter
                helpTitle: "작업 제어"
                helpText: "경로 작성 후 [도면 전송]으로 서버에 저장하고, 확인 뒤 [그림그리기 시작]을 누릅니다. 작업 취소는 현재 경로를 폐기하고, ESTOP은 로봇을 즉시 정지시킵니다."
            }
        ]

        Flickable {
            id: rightFlick
            anchors.fill: parent
            anchors.margins: 10
            contentHeight: rightInner.implicitHeight
            clip: true
            boundsBehavior: Flickable.StopAtBounds
            flickableDirection: Flickable.VerticalFlick

            Column {
                id: rightInner
                width: rightFlick.width
                spacing: 10

                // 로봇 상태
                Rectangle {
                    width: parent.width
                    height: robotCol.implicitHeight + 28
                    radius: Theme.radius
                    color: Theme.surface
                    border.color: Theme.stroke
                    border.width: 1
                    Column {
                        id: robotCol
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 14
                        spacing: 10
                        Text {
                            text: "로봇 상태"
                            color: Theme.sub
                            font.pixelSize: 12
                            font.bold: true
                            font.family: Theme.fontFamily
                        }
                        KvRow {
                            k: "연결"
                            v: Backend.robotOnline ? "연결됨" : "오프라인"
                            vColor: Backend.robotOnline ? Theme.success : Theme.danger
                        }
                        KvRow {
                            k: "상태"
                            v: Backend.robotState
                            vColor: Backend.robotState === "ESTOPPED" ? Theme.danger
                                  : (Backend.robotState === "MOVING" ? Theme.warn : Theme.text)
                        }
                        KvRow {
                            k: "마킹"
                            v: Backend.painting ? "활성 (도장 중)" : "대기"
                            vColor: Backend.painting ? Theme.success : Theme.sub
                        }
                        KvRow {
                            k: "위치"
                            v: Backend.poseValid
                               ? Backend.poseX.toFixed(2) + ", " + Backend.poseY.toFixed(2) + " m"
                               : "—"
                        }
                        KvRow {
                            k: "현재 노드"
                            v: Backend.waypointCount > 0
                               ? (Backend.waypointIndex + 1) + " / " + Backend.waypointCount
                               : "—"
                        }
                        KvRow {
                            k: "요약"
                            v: Backend.robotStatus
                        }
                    }
                }

                // 작업 정보
                Rectangle {
                    width: parent.width
                    height: workCol.implicitHeight + 28
                    radius: Theme.radius
                    color: Theme.surface
                    border.color: Theme.stroke
                    border.width: 1
                    Column {
                        id: workCol
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 14
                        spacing: 10
                            Row {
                                width: parent.width
                                spacing: 6
                                Text {
                                    text: "작업 정보"
                                    color: Theme.sub
                                    font.pixelSize: 12
                                    font.bold: true
                                    font.family: Theme.fontFamily
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                HelpIcon {
                                    anchors.verticalCenter: parent.verticalCenter
                                    helpTitle: "작업 단계"
                                    helpText: "작도 → 도면 전송 → 작업 시작 순서입니다. 도면 전송만으로 로봇은 움직이지 않습니다. 작업 시작 후 서버가 시작점 접근과 도색을 이어서 수행합니다."
                                }
                            }
                        // 단계 표시 — 지금 어디쯤인지 한눈에
                        Row {
                            width: parent.width
                            spacing: 4
                            Repeater {
                                // 접근과 도색은 서버가 한 덩어리로 자동 진행하므로
                                // 단계를 나눠 보여주지 않는다 (통지가 없어 알 수도 없다)
                                model: [
                                    { t: "작도", on: ["drawing", "idle", "ready"] },
                                    { t: "도면 전송", on: ["sent"] },
                                    { t: "접근+도색", on: ["running"] },
                                    { t: "완료", on: ["done"] }
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    width: (parent.width - 12) / 4
                                    height: 22
                                    radius: 4
                                    property bool active: modelData.on.indexOf(Backend.phase) >= 0
                                    color: active ? Theme.accent : Theme.elevated
                                    Text {
                                        anchors.centerIn: parent
                                        text: modelData.t
                                        color: parent.active ? "#FFFFFF" : Theme.muted
                                        font.pixelSize: 10
                                        font.bold: parent.active
                                        font.family: Theme.fontFamily
                                    }
                                }
                            }
                        }
                        KvRow { k: "작업"; v: Backend.workName }
                        Column {
                            width: parent.width
                            spacing: 4
                            Row {
                                width: parent.width
                                Text {
                                    text: "진행률"
                                    color: Theme.muted
                                    font.pixelSize: 12
                                    font.family: Theme.fontFamily
                                    width: parent.width * 0.42
                                }
                                Text {
                                    text: Backend.jobPercent + "%"
                                    color: Theme.accent
                                    font.pixelSize: 12
                                    font.bold: true
                                    font.family: Theme.fontFamily
                                    width: parent.width * 0.58
                                    horizontalAlignment: Text.AlignRight
                                }
                            }
                            Rectangle {
                                width: parent.width
                                height: 6
                                radius: 3
                                color: Theme.elevated
                                Rectangle {
                                    height: parent.height
                                    radius: 3
                                    width: parent.width * Backend.jobProgress
                                    color: Theme.accent
                                    Behavior on width { NumberAnimation { duration: 120 } }
                                }
                            }
                        }
                        KvRow {
                            k: "도색 거리"
                            v: Backend.pathLengthM > 0
                               ? Backend.pathLengthM.toFixed(2) + " m" : "—"
                        }
                        // 도형 사이를 칠하지 않고 지나가는 구간. 0 이면 한붓그리기.
                        KvRow {
                            k: "빈 이동"
                            v: Backend.travelLengthM > 0
                               ? Backend.travelLengthM.toFixed(2) + " m" : "없음"
                        }
                        // 진행 중이면 실측 기반 남은 시간, 아니면 로봇 사양 기반 최소 예상치
                        KvRow {
                            k: Backend.jobActive ? "남은 시간" : "예상 소요(최소)"
                            v: Backend.jobActive ? Backend.etaText : Backend.planTimeText
                        }
                        KvRow { k: "경과 시간"; v: Backend.elapsedText }
                        KvRow {
                            k: "남은 거리"
                            v: (Backend.jobActive || Backend.phase === "done") && Backend.pathLengthM > 0
                               ? Backend.remainingM.toFixed(2) + " m" : "—"
                        }

                        // ── 테스트 모드 시뮬레이션 ──
                        // 로봇 없이 동작 시퀀스를 그대로 재생한다. 지금 어느 동작을 하는
                        // 중인지(후진·제자리회전·노즐)를 보여줘야 미리보기가 쓸모 있다.
                        Rectangle {
                            visible: Backend.testMode
                            width: parent.width
                            height: simCol.implicitHeight + 16
                            radius: 6
                            color: Theme.elevated
                            border.color: Backend.simRunning ? Theme.accent : Theme.stroke
                            border.width: 1
                            Column {
                                id: simCol
                                anchors.left: parent.left
                                anchors.right: parent.right
                                anchors.top: parent.top
                                anchors.margins: 8
                                spacing: 6
                                Row {
                                    width: parent.width
                                    Text {
                                        text: "시뮬레이션"
                                        color: Theme.sub
                                        font.pixelSize: 11
                                        font.bold: true
                                        font.family: Theme.fontFamily
                                        width: parent.width * 0.42
                                    }
                                    Text {
                                        text: Backend.simRunning ? Backend.simPhase : "대기"
                                        color: Backend.simRunning ? Theme.accent : Theme.muted
                                        font.pixelSize: 11
                                        font.bold: Backend.simRunning
                                        font.family: Theme.fontFamily
                                        width: parent.width * 0.58
                                        horizontalAlignment: Text.AlignRight
                                        elide: Text.ElideRight
                                    }
                                }
                                Row {
                                    width: parent.width
                                    spacing: 4
                                    Text {
                                        text: "재생 배속"
                                        color: Theme.muted
                                        font.pixelSize: 11
                                        font.family: Theme.fontFamily
                                        anchors.verticalCenter: parent.verticalCenter
                                        width: parent.width * 0.42
                                    }
                                    Repeater {
                                        model: [1, 2, 4, 8]
                                        delegate: Rectangle {
                                            required property int modelData
                                            width: (parent.width * 0.58 - 12) / 4
                                            height: 22
                                            radius: 4
                                            property bool on: Math.abs(Backend.simSpeedFactor - modelData) < 0.01
                                            color: on ? Theme.accent : Theme.surface
                                            border.color: on ? Theme.accent : Theme.stroke
                                            border.width: 1
                                            Text {
                                                anchors.centerIn: parent
                                                text: modelData + "×"
                                                color: parent.on ? "#FFFFFF" : Theme.sub
                                                font.pixelSize: 10
                                                font.bold: parent.on
                                                font.family: Theme.fontFamily
                                            }
                                            MouseArea {
                                                anchors.fill: parent
                                                cursorShape: Qt.PointingHandCursor
                                                onClicked: Backend.simSpeedFactor = modelData
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // ── 로봇 동작 시퀀스 (SERVER_PROTOCOL 의 PATH 미리보기) ──
                // 서버가 BLUEPRINT 로 만들어 로봇에 보낼 MOVE/TURN 을 같은 부호 규약으로
                // 미리 계산해 보여준다. 꼭짓점마다 어느 쪽으로 몇 도 도는지가 여기서 확정된다.
                Rectangle {
                    id: planCard
                    property bool expanded: false
                    width: parent.width
                    height: planCol.implicitHeight + 24
                    radius: Theme.radius
                    color: Theme.surface
                    border.color: Theme.stroke
                    border.width: 1
                    visible: Backend.motionPlan.length > 0

                    Column {
                        id: planCol
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 12
                        spacing: 8

                        Row {
                            width: parent.width
                            spacing: 6
                            Row {
                                spacing: 5
                                anchors.verticalCenter: parent.verticalCenter
                                Text {
                                    text: "로봇 동작 시퀀스"
                                    color: Theme.sub
                                    font.pixelSize: 12
                                    font.bold: true
                                    font.family: Theme.fontFamily
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                                HelpIcon {
                                    anchors.verticalCenter: parent.verticalCenter
                                    helpTitle: "동작 시퀀스"
                                    helpText: "Qt가 서버에 보낼 경로를 MOVE, TURN, ARC, 노즐 동작으로 미리 보여줍니다. 원과 유효한 원호는 여러 직선이 아니라 ARC 한 동작이어야 합니다. 펜 오프셋과 실제 바퀴 제어는 서버와 로봇이 담당합니다."
                                }
                            }
                            Rectangle {
                                width: seqTag.implicitWidth + 12
                                height: 18
                                radius: 9
                                color: Theme.accentSoft
                                anchors.verticalCenter: parent.verticalCenter
                                Text {
                                    id: seqTag
                                    anchors.centerIn: parent
                                    text: Backend.motionPlan.length + "동작 · "
                                          + Backend.turnCount + "회전"
                                    color: Theme.accentDim
                                    font.pixelSize: 10
                                    font.bold: true
                                    font.family: Theme.fontFamily
                                }
                            }
                            Item { width: parent.width - 190; height: 1 }
                            AppButton {
                                width: 46; height: 22
                                text: planCard.expanded ? "접기" : "펼치기"
                                anchors.verticalCenter: parent.verticalCenter
                                onClicked: planCard.expanded = !planCard.expanded
                            }
                        }

                        // 접힌 상태에서는 시퀀스를 한 줄 띠로 요약해서 보여준다
                        Row {
                            visible: !planCard.expanded
                            width: parent.width
                            spacing: 2
                            Repeater {
                                model: Math.min(Backend.motionPlan.length, 22)
                                delegate: Rectangle {
                                    required property int index
                                    readonly property var seg: Backend.motionPlan[index]
                                    width: (planCol.width - 42) / 22
                                    height: 16
                                    radius: 2
                                    // 노즐 = 초록, 회전 = 파랑/주황, 도색 직진 = 강조색,
                                    // 칠하지 않는 이동·후진 = 흐린 회색
                                    color: seg.op === "NOZZLE"
                                           ? "#66BB6A"
                                           : seg.op === "TURN"
                                             ? (seg.angle > 0 ? "#4FC3F7" : "#FFB74D")
                                             : seg.op === "ARC"
                                               ? "#B39DDB"
                                               : (seg.paint ? Theme.accent : Theme.muted)
                                    opacity: seg.op === "MOVE" && !seg.paint ? 0.45 : 0.85
                                    ToolTip.visible: segHover.hovered
                                    ToolTip.text: seg.op === "NOZZLE"
                                        ? (seg.down ? "노즐 내림" : "노즐 올림")
                                        : seg.op === "TURN"
                                          ? ((seg.angle > 0 ? "좌회전 " : "우회전 ")
                                             + Math.abs(seg.angle).toFixed(1) + "° · "
                                             + seg.speed.toFixed(0) + "°/s")
                                          : seg.op === "ARC"
                                            ? ("곡선 " + (seg.dir === "left" ? "좌 " : "우 ")
                                               + seg.angle.toFixed(0) + "° · R "
                                               + seg.radius.toFixed(2) + " m · "
                                               + seg.dist.toFixed(2) + " m")
                                            : ((seg.dist < 0 ? "후진 "
                                                : seg.paint ? "도색 " : "이동(도색 안 함) ")
                                               + Math.abs(seg.dist).toFixed(3) + " m · "
                                               + seg.speed.toFixed(2) + " m/s")
                                    HoverHandler { id: segHover }
                                }
                            }
                        }

                        // 펼친 상태 — 실제 PATH 세그먼트 목록
                        Column {
                            visible: planCard.expanded
                            width: parent.width
                            spacing: 3
                            Repeater {
                                model: Math.min(Backend.motionPlan.length, 40)
                                delegate: Row {
                                    required property int index
                                    readonly property var seg: Backend.motionPlan[index]
                                    width: planCol.width
                                    spacing: 6
                                    Text {
                                        width: 20
                                        text: (index + 1)
                                        color: Theme.muted
                                        font.pixelSize: 10
                                        font.family: Theme.fontFamily
                                    }
                                    Rectangle {
                                        width: 42; height: 15; radius: 3
                                        color: seg.op === "NOZZLE"
                                               ? "#1E3A22"
                                               : seg.op === "TURN"
                                                 ? (seg.angle > 0 ? "#1D3E52" : "#4A3620")
                                                 : seg.op === "ARC" ? "#332B4A"
                                                 : Theme.elevated
                                        Text {
                                            anchors.centerIn: parent
                                            // 같은 MOVE 라도 도색/이동/후진은 눈에 바로 구분돼야 한다
                                            text: seg.op === "NOZZLE" ? "노즐"
                                                  : seg.op === "TURN" ? "TURN"
                                                  : seg.op === "ARC" ? "곡선"
                                                  : seg.dist < 0 ? "후진"
                                                  : seg.paint ? "MOVE" : "이동"
                                            color: seg.op === "NOZZLE"
                                                   ? "#66BB6A"
                                                   : seg.op === "TURN"
                                                     ? (seg.angle > 0 ? "#4FC3F7" : "#FFB74D")
                                                     : seg.op === "ARC" ? "#B39DDB"
                                                     : (seg.paint ? Theme.sub : Theme.muted)
                                            font.pixelSize: 9
                                            font.bold: true
                                            font.family: Theme.fontFamily
                                        }
                                    }
                                    Text {
                                        text: seg.op === "NOZZLE"
                                            ? (seg.down ? "내림" : "올림")
                                            : seg.op === "TURN"
                                              ? ((seg.angle > 0 ? "좌 " : "우 ")
                                                 + Math.abs(seg.angle).toFixed(1) + "°")
                                              : seg.op === "ARC"
                                                ? ((seg.dir === "left" ? "좌 " : "우 ")
                                                   + seg.angle.toFixed(0) + "° · R "
                                                   + seg.radius.toFixed(2) + " · "
                                                   + seg.dist.toFixed(2) + " m")
                                                : (Math.abs(seg.dist).toFixed(3) + " m")
                                        color: seg.op === "MOVE" && !seg.paint
                                               ? Theme.muted : Theme.text
                                        font.pixelSize: 10
                                        font.family: Theme.fontFamily
                                    }
                                    Text {
                                        text: seg.op === "NOZZLE"
                                              ? ("꼭짓점 " + seg.vertex)
                                              : ("방위 " + seg.heading.toFixed(0) + "°")
                                        color: Theme.muted
                                        font.pixelSize: 10
                                        font.family: Theme.fontFamily
                                    }
                                    Text {
                                        visible: seg.op !== "NOZZLE"
                                        text: seg.op === "TURN"
                                              ? (seg.speed.toFixed(0) + "°/s")
                                              : (seg.speed.toFixed(2) + " m/s")
                                        color: seg.op === "MOVE" && seg.paint
                                               ? Theme.accentDim : Theme.muted
                                        font.pixelSize: 10
                                        font.family: Theme.fontFamily
                                    }
                                }
                            }
                            Text {
                                visible: Backend.motionPlan.length > 40
                                text: "… 외 " + (Backend.motionPlan.length - 40) + "개"
                                color: Theme.muted
                                font.pixelSize: 10
                                font.family: Theme.fontFamily
                            }
                        }
                    }
                }

                // 주 동작
                AppButton {
                    width: parent.width
                    height: 44
                    accent: !Backend.jobActive
                    text: page.primaryText()
                    enabled: !Backend.jobActive
                    onClicked: page.primaryAction()
                }

                // 진행 중 제어 — 중단 / 수정
                Row {
                    width: parent.width
                    spacing: 8
                    visible: Backend.jobActive || Backend.canEditMission
                    AppButton {
                        width: (parent.width - 8) / 2
                        height: 38
                        danger: true
                        outline: true
                        text: Backend.abortPending ? "취소 요청 중…" : "작업 취소"
                        visible: Backend.jobActive
                        enabled: !Backend.abortPending
                        ToolTip.visible: hovered
                        ToolTip.text: "서버와 로봇의 현재 실행 경로를 폐기합니다. 즉시 정지는 아래 ESTOP을 사용하세요."
                        onClicked: cancelConfirmPopup.open()
                    }
                    AppButton {
                        width: Backend.jobActive
                               ? (parent.width - 8) / 2 : parent.width
                        height: 38
                        outline: true
                        text: "경로 수정"
                        enabled: Backend.canEditMission && !Backend.jobActive
                        ToolTip.visible: hovered
                        ToolTip.text: Backend.jobActive
                            ? "먼저 작업을 취소하세요"
                            : "보냈던 경로를 다시 불러와 고칩니다"
                        onClicked: Backend.editMission()
                    }
                }

                AppButton {
                    width: parent.width
                    height: 44
                    danger: Backend.robotState !== "ESTOPPED"
                    accent: Backend.robotState === "ESTOPPED"
                    text: Backend.robotState === "ESTOPPED" ? "ESTOP 해제 (RESUME)" : "ESTOP 비상정지"
                    onClicked: Backend.toggleEstop()
                }

                Row {
                    width: parent.width
                    spacing: 8
                    AppButton {
                        width: (parent.width - 8) / 2
                        text: "로그아웃"
                        onClicked: logoutPopup.open()
                    }
                    AppButton {
                        width: (parent.width - 8) / 2
                        text: "종료"
                        onClicked: quitPopup.open()
                    }
                }
                Item { width: 1; height: 4 }
            }
        }
    }

    // ── 팝업들 ──────────────────────────────────────────────────────
    JobHistoryDialog { id: historyDialog }

    Popup {
        id: tutorialPopup
        property int step: 0
        readonly property var steps: [
            {
                title: "작업 채널 선택",
                body: "4채널 화면에서 로봇을 볼 채널의 [작업하기]를 누릅니다. 영상이 없으면 설정에서 카메라 IP 또는 RTSP 주소를 먼저 확인하세요.",
                action: "확인할 것: CCTV 연결 상태와 올바른 채널",
                visual: "예시 화면 · 작업 채널 선택",
                image: "qrc:/assets/tutorial_step_1_channel.jpg"
            },
            {
                title: "Top View 확인",
                body: "Top View의 보정 상태와 축척을 확인합니다. 도면 좌표는 이 바닥 평면 기준으로 서버에 전송되므로, 카메라가 움직였다면 먼저 다시 캘리브레이션해야 합니다.",
                action: "확인할 것: 바닥 기준점과 축척",
                visual: "Top View · 보정된 바닥 평면",
                image: "qrc:/assets/tutorial_step_2_calibration.jpg"
            },
            {
                title: "경로 작성",
                body: "프리셋을 끌어놓거나 [경로 추가]로 점을 찍습니다. 크기 조절은 기본적으로 비율을 유지하며 Shift를 누른 동안만 자유 변형합니다. 도색 반지름은 200 mm보다 작게 줄어들지 않습니다.",
                action: "곡선이 빨간색이면 크기를 키우세요",
                visual: "도색 경로 · R ≥ 200 mm",
                image: "qrc:/assets/tutorial_step_3_path.jpg"
            },
            {
                title: "경로 검토",
                body: "오른쪽 동작 시퀀스에서 직선은 MOVE, 곡선은 ARC로 생성됐는지 확인합니다. 원 하나는 ARC 한 동작이어야 하며, 점 개수가 많다고 MOVE가 늘어나면 전송 전에 경로를 수정합니다.",
                action: "확인할 것: 거리, ARC 반지름, 동작 수",
                visual: "경로 미리보기 · MOVE 1 · ARC 1",
                image: "qrc:/assets/tutorial_step_4_review.jpg"
            },
            {
                title: "전송하고 시작",
                body: "[도면 전송]은 경로만 서버에 저장합니다. 서버 확인이 끝난 뒤 [그림그리기 시작]을 눌러야 로봇이 움직입니다. 정상 중단은 [작업 취소], 즉시 정지는 [ESTOP]을 사용합니다.",
                action: "시작 전 로봇 주변과 노즐 상태를 확인하세요",
                visual: "도면 전송 → 서버 확인 → 작업 시작",
                image: "qrc:/assets/tutorial_step_5_execute.jpg"
            }
        ]
        readonly property var currentStep: steps[step]
        modal: true
        anchors.centerIn: Overlay.overlay
        width: Math.min(620, page.width - 32)
        padding: 20
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        onAboutToShow: step = 0
        background: Rectangle {
            radius: 6
            color: Theme.surface
            border.width: 1
            border.color: Theme.stroke
        }
        contentItem: Column {
            width: tutorialPopup.width - 40
            spacing: 18
            Row {
                width: parent.width
                spacing: 10
                Rectangle {
                    width: 30; height: 30; radius: 15
                    color: Theme.accent
                    Text {
                        anchors.centerIn: parent
                        text: tutorialPopup.step + 1
                        color: "#FFFFFF"
                        font.pixelSize: 13
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                }
                Column {
                    width: parent.width - 40
                    spacing: 2
                    Text {
                        width: parent.width
                        text: "Road Painter 사용 순서"
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                    }
                    Text {
                        width: parent.width
                        text: tutorialPopup.currentStep.title
                        color: Theme.text
                        font.pixelSize: 17
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                }
            }
            Text {
                width: parent.width
                text: tutorialPopup.currentStep.visual
                color: Theme.sub
                font.pixelSize: 11
                font.bold: true
                font.family: Theme.fontFamily
            }
            Rectangle {
                width: parent.width
                height: Math.min(250, width * 0.46)
                radius: 5
                clip: true
                color: Theme.panel
                border.width: 1
                border.color: Theme.stroke
                Image {
                    anchors.fill: parent
                    source: tutorialPopup.currentStep.image
                    fillMode: Image.PreserveAspectFit
                    smooth: true
                    mipmap: true
                }
            }
            Text {
                width: parent.width
                text: tutorialPopup.currentStep.body
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
                lineHeight: 1.35
                wrapMode: Text.WordWrap
            }
            Rectangle {
                width: parent.width
                height: actionText.implicitHeight + 20
                radius: 5
                color: Theme.accentSoft
                border.width: 1
                border.color: Theme.accent
                Text {
                    id: actionText
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.margins: 10
                    text: tutorialPopup.currentStep.action
                    color: Theme.accentDim
                    font.pixelSize: 12
                    font.bold: true
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }
            }
            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 6
                Repeater {
                    model: tutorialPopup.steps.length
                    Rectangle {
                        required property int index
                        width: tutorialPopup.step === index ? 18 : 6
                        height: 6
                        radius: 3
                        color: tutorialPopup.step === index ? Theme.accent : Theme.stroke
                        Behavior on width { NumberAnimation { duration: 120 } }
                    }
                }
            }
            Row {
                anchors.right: parent.right
                spacing: 8
                AppButton {
                    visible: tutorialPopup.step > 0
                    text: "이전"
                    onClicked: tutorialPopup.step--
                }
                AppButton {
                    accent: true
                    text: tutorialPopup.step + 1 === tutorialPopup.steps.length ? "완료" : "다음"
                    onClicked: {
                        if (tutorialPopup.step + 1 === tutorialPopup.steps.length) tutorialPopup.close()
                        else tutorialPopup.step++
                    }
                }
            }
        }
    }

    Popup {
        id: sendConfirmPopup
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 380
        padding: 20
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Column {
            width: 340
            spacing: 14
            Text {
                text: "경로 전송"
                color: Theme.text
                font.pixelSize: 16
                font.bold: true
                font.family: Theme.fontFamily
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "도면을 서버에 저장합니다. 이 단계에서는 로봇이 움직이지 않습니다 — "
                    + "실제 작업은 [그림그리기 시작]을 눌러야 시작됩니다."
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
            }
            Text {
                text: "경로 " + sendConfirmPopup.pointCount + "점 · "
                      + (Backend.pathLengthM > 0 ? Backend.pathLengthM.toFixed(2) + " m" : "")
                      + (Backend.testMode ? " · 테스트" : "")
                color: Theme.muted
                font.pixelSize: 12
                font.family: Theme.fontFamily
                width: parent.width
                wrapMode: Text.WordWrap
            }
            Row {
                anchors.right: parent.right
                spacing: 6
                AppButton { text: "취소"; onClicked: sendConfirmPopup.close() }
                AppButton {
                    accent: true
                    text: "전송"
                    onClicked: {
                        sendConfirmPopup.close()
                        Backend.commitDrawing()
                    }
                }
            }
        }
        property int pointCount: 0
        onAboutToShow: pointCount = Backend.pathPointCount()
    }

    // 그림그리기 시작 — 여기서부터는 서버가 접근·도색·완료까지 자동으로 진행한다.
    Popup {
        id: startConfirmPopup
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 400
        padding: 20
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Column {
            width: 360
            spacing: 14
            Text {
                text: "그림그리기 시작"
                color: Theme.text
                font.pixelSize: 16
                font.bold: true
                font.family: Theme.fontFamily
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "누르는 즉시 로봇이 시작점으로 이동하고, 도착하면 이어서 도색까지 "
                    + "자동으로 진행됩니다. 급히 세워야 하면 [비상 정지]를 누르세요."
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "경로 " + Backend.waypointCount + "점 · "
                      + (Backend.pathLengthM > 0 ? Backend.pathLengthM.toFixed(2) + " m" : "")
                      + " · 직진 " + (Backend.motionPlan.length - Backend.turnCount)
                      + "회 · 회전 " + Backend.turnCount + "회"
                      + (Backend.testMode ? " · 테스트" : "")
                color: Theme.muted
                font.pixelSize: 12
                font.family: Theme.fontFamily
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                visible: !Backend.robotOnline && !Backend.testMode
                text: "⚠ 로봇이 접속돼 있지 않습니다 — 서버가 robot_offline 로 거절할 수 있습니다."
                color: Theme.danger
                font.pixelSize: 12
                font.family: Theme.fontFamily
            }
            Row {
                anchors.right: parent.right
                spacing: 6
                AppButton { text: "취소"; onClicked: startConfirmPopup.close() }
                AppButton {
                    accent: true
                    text: "시작"
                    onClicked: {
                        startConfirmPopup.close()
                        Backend.startPainting()
                    }
                }
            }
        }
    }

    Popup {
        id: cancelConfirmPopup
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 380
        padding: 20
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Column {
            width: 340
            spacing: 14
            Text { text: "작업 취소"; color: Theme.text; font.pixelSize: 16; font.bold: true; font.family: Theme.fontFamily }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "현재 작업을 중단하고 서버와 로봇이 실행 중인 경로를 폐기합니다."
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
            }
            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "취소 응답을 받은 뒤 같은 도면을 다시 시작하거나 경로를 수정할 수 있습니다. "
                    + "위험 상황에서 즉시 멈춰야 할 때는 별도의 ESTOP 버튼을 사용하세요."
                color: Theme.muted
                font.pixelSize: 12
                font.family: Theme.fontFamily
            }
            Row {
                anchors.right: parent.right
                spacing: 6
                AppButton { text: "계속 진행"; onClicked: cancelConfirmPopup.close() }
                AppButton {
                    danger: true
                    text: "작업 취소"
                    onClicked: { cancelConfirmPopup.close(); Backend.cancelJob() }
                }
            }
        }
    }

    Popup {
        id: logoutPopup
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 320
        padding: 20
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Column {
            width: 280
            spacing: 14
            Text { text: "로그아웃"; color: Theme.text; font.pixelSize: 16; font.bold: true; font.family: Theme.fontFamily }
            Text { text: "로그아웃 하시겠습니까?"; color: Theme.sub; font.pixelSize: 13; font.family: Theme.fontFamily }
            Row {
                anchors.right: parent.right
                spacing: 6
                AppButton { text: "취소"; onClicked: logoutPopup.close() }
                AppButton { danger: true; text: "로그아웃"; onClicked: { logoutPopup.close(); Backend.logout() } }
            }
        }
    }

    Popup {
        id: quitPopup
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 320
        padding: 20
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Column {
            width: 280
            spacing: 14
            Text { text: "종료"; color: Theme.text; font.pixelSize: 16; font.bold: true; font.family: Theme.fontFamily }
            Text { text: "애플리케이션을 종료할까요?"; color: Theme.sub; font.pixelSize: 13; font.family: Theme.fontFamily }
            Row {
                anchors.right: parent.right
                spacing: 6
                AppButton { text: "취소"; onClicked: quitPopup.close() }
                AppButton { danger: true; text: "종료"; onClicked: Qt.quit() }
            }
        }
    }

    Popup {
        id: settingsPopup
        property int tabIndex: 0
        property int calibrationChannel: Backend.workingChannel > 0
            ? Backend.workingChannel
            : (Backend.highlightedChannel > 0 ? Backend.highlightedChannel : 1)
        property bool manualCalibrationExpanded: false
        // 캘리 방식 선택. 기본은 오도메트리 주행(서버 요청서 20260813).
        property bool calibOdometry: true
        property real dragStartPopupX: 0
        property real dragStartPopupY: 0
        property real dragStartPointerX: 0
        property real dragStartPointerY: 0
        onAboutToShow: {
            calibrationChannel = Backend.workingChannel > 0
                ? Backend.workingChannel
                : (Backend.highlightedChannel > 0 ? Backend.highlightedChannel : 1)
            settingsScroll.contentY = 0
            x = Math.max(12, (Overlay.overlay.width - width) / 2)
            y = Math.max(12, (Overlay.overlay.height - height) / 2)
        }
        modal: true
        width: Math.min(560, Overlay.overlay.width - 24)
        height: Math.min(760, Overlay.overlay.height - 24)
        padding: 16
        background: Rectangle {
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
        }
        contentItem: Flickable {
            id: settingsScroll
            clip: true
            contentWidth: width
            contentHeight: settingsContent.implicitHeight
            boundsBehavior: Flickable.StopAtBounds
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }

          Column {
            id: settingsContent
            spacing: 10
            width: settingsScroll.width - 12

            Item {
                width: parent.width
                height: 30

                MouseArea {
                    anchors.fill: parent
                    z: 0
                    cursorShape: Qt.OpenHandCursor
                    hoverEnabled: true
                    onPressed: function(mouse) {
                        cursorShape = Qt.ClosedHandCursor
                        const p = mapToItem(Overlay.overlay, mouse.x, mouse.y)
                        settingsPopup.dragStartPointerX = p.x
                        settingsPopup.dragStartPointerY = p.y
                        settingsPopup.dragStartPopupX = settingsPopup.x
                        settingsPopup.dragStartPopupY = settingsPopup.y
                    }
                    onPositionChanged: function(mouse) {
                        if (!pressed) return
                        const p = mapToItem(Overlay.overlay, mouse.x, mouse.y)
                        const maxX = Math.max(12, Overlay.overlay.width - settingsPopup.width - 12)
                        const maxY = Math.max(12, Overlay.overlay.height - settingsPopup.height - 12)
                        settingsPopup.x = Math.max(12, Math.min(maxX,
                            settingsPopup.dragStartPopupX + p.x - settingsPopup.dragStartPointerX))
                        settingsPopup.y = Math.max(12, Math.min(maxY,
                            settingsPopup.dragStartPopupY + p.y - settingsPopup.dragStartPointerY))
                    }
                    onReleased: cursorShape = Qt.OpenHandCursor
                    onCanceled: cursorShape = Qt.OpenHandCursor
                    ToolTip.visible: containsMouse && !pressed
                    ToolTip.text: "끌어서 설정 창 이동"
                }
                Text {
                    z: 1
                    anchors.left: parent.left
                    anchors.verticalCenter: parent.verticalCenter
                    text: "설정"
                    color: Theme.text
                    font.pixelSize: 16
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                HelpIcon {
                    id: settingsHelpIcon
                    z: 2
                    anchors.left: parent.left
                    anchors.leftMargin: 42
                    anchors.verticalCenter: parent.verticalCenter
                    helpTitle: "설정"
                    helpText: "영상은 화면 표시와 카메라 연결, 캘리브는 Top View 좌표 보정, 서버는 로그인 서버 주소를 설정합니다. 연결 정보를 바꾸면 적용 후 미리보기가 다시 열릴 수 있습니다."
                }
                AppButton {
                    z: 2
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    width: 58
                    height: 28
                    text: "닫기"
                    onClicked: settingsPopup.close()
                }
            }

            Row {
                spacing: 0
                Repeater {
                    model: ["영상", "캘리브", "서버"]
                    delegate: Rectangle {
                        width: 80
                        height: 30
                        color: settingsPopup.tabIndex === index ? Theme.accentSoft : "transparent"
                        radius: 6
                        border.color: settingsPopup.tabIndex === index ? Theme.accent : Theme.stroke
                        border.width: 1
                        Text {
                            anchors.centerIn: parent
                            text: modelData
                            color: settingsPopup.tabIndex === index ? Theme.accentDim : Theme.sub
                            font.pixelSize: 12
                            font.bold: settingsPopup.tabIndex === index
                            font.family: Theme.fontFamily
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: settingsPopup.tabIndex = index
                        }
                    }
                }
            }

            // ── 영상 탭 ──
            Column {
                width: parent.width
                spacing: 10
                visible: settingsPopup.tabIndex === 0

                component FilterRow: Column {
                    id: filterRow
                    width: 488
                    property alias value: sl.value
                    property string label: ""
                    property real from: -100
                    property real to: 100
                    property real initialValue: 0
                    function setFromText(t) {
                        const n = Number(t)
                        if (!isFinite(n)) return
                        sl.value = Math.max(from, Math.min(to, Math.round(n)))
                        settingsPopup.applyFilters()
                        numField.text = String(Math.round(sl.value))
                    }
                    Row {
                        width: parent.width
                        spacing: 8
                        Text {
                            text: filterRow.label
                            color: Theme.sub
                            font.pixelSize: 12
                            font.family: Theme.fontFamily
                            width: 56
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Slider {
                            id: sl
                            width: parent.width - 56 - 72 - 16
                            from: filterRow.from
                            to: filterRow.to
                            stepSize: 1
                            value: filterRow.initialValue
                            onValueChanged: {
                                if (!numField.activeFocus)
                                    numField.text = String(Math.round(value))
                            }
                            anchors.verticalCenter: parent.verticalCenter
                            onMoved: {
                                numField.text = String(Math.round(value))
                                settingsPopup.applyFilters()
                            }
                        }
                        TextField {
                            id: numField
                            width: 72
                            height: 28
                            text: String(Math.round(filterRow.initialValue))
                            color: Theme.text
                            horizontalAlignment: Text.AlignHCenter
                            font.family: Theme.fontFamily
                            font.pixelSize: 12
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            anchors.verticalCenter: parent.verticalCenter
                            onEditingFinished: filterRow.setFromText(text)
                            onAccepted: filterRow.setFromText(text)
                            background: Rectangle {
                                radius: 4
                                color: Theme.panel
                                border.width: 1
                                border.color: numField.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                    }
                }
                FilterRow { id: fBright; label: "밝기"; initialValue: Backend.videoBrightness }
                FilterRow { id: fContrast; label: "대비"; initialValue: Backend.videoContrast }
                FilterRow { id: fSharpen; label: "선명도"; from: 0; to: 100; initialValue: Backend.videoSharpen }
                FilterRow { id: fSat; label: "채도"; initialValue: Backend.videoSaturation }

                Row {
                    width: parent.width
                    spacing: 8
                    AppButton {
                        text: "권장 기본값"
                        onClicked: settingsPopup.resetVideoFilters()
                    }
                    Text {
                        text: "밝기 45 · 대비 8 · 선명도 0 · 채도 0"
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    AppButton {
                        text: Backend.arucoOverlay ? "ArUco 검출: 켬" : "ArUco 검출: 끔"
                        accent: Backend.arucoOverlay
                        onClicked: Backend.arucoOverlay = !Backend.arucoOverlay
                    }
                    Text {
                        text: Backend.arucoSummary.length ? Backend.arucoSummary : "검출된 마커 없음"
                        color: Theme.muted
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text {
                        text: "펜촉 폭"
                        color: Theme.sub
                        font.pixelSize: 12
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    AppButton {
                        width: 44; height: 30
                        text: "50"
                        accent: Math.abs(Backend.strokeWidthMm - 50) < 0.01
                        onClicked: Backend.strokeWidthMm = 50
                        ToolTip.visible: hovered
                        ToolTip.text: "50 mm 직각 팁"
                    }
                    AppButton {
                        width: 44; height: 30
                        text: "60"
                        accent: Math.abs(Backend.strokeWidthMm - 60) < 0.01
                        onClicked: Backend.strokeWidthMm = 60
                        ToolTip.visible: hovered
                        ToolTip.text: "60 mm 실측 도장 폭 (기본)"
                    }
                    Text {
                        text: "직접"
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    TextField {
                        id: strokeField
                        width: 62; height: 30
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        ToolTip.visible: hovered
                        ToolTip.text: "직접 사용할 펜촉 폭을 mm로 입력"
                        Component.onCompleted: text = Backend.strokeWidthMm.toFixed(0)
                        onAccepted: {
                            Backend.strokeWidthMm = Number(text)
                            text = Backend.strokeWidthMm.toFixed(0)
                        }
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1
                            border.color: strokeField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    Text {
                        text: "mm"
                        color: Theme.muted
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    AppButton {
                        height: 30
                        text: "적용"
                        onClicked: {
                            Backend.strokeWidthMm = Number(strokeField.text)
                            strokeField.text = Backend.strokeWidthMm.toFixed(0)
                        }
                    }
                    Connections {
                        target: Backend
                        function onStrokeWidthChanged() {
                            if (!strokeField.activeFocus)
                                strokeField.text = Backend.strokeWidthMm.toFixed(0)
                        }
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text {
                        text: "중심-펜 거리"
                        color: Theme.sub
                        font.pixelSize: 12
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    TextField {
                        id: penDisplayOffsetField
                        width: 72; height: 30
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        ToolTip.visible: hovered
                        ToolTip.text: "TopView에서 마커 중심과 펜 위치 사이의 표시 거리"
                        Component.onCompleted: text = Backend.penDisplayOffsetMm.toFixed(0)
                        onAccepted: {
                            Backend.penDisplayOffsetMm = Number(text)
                            text = Backend.penDisplayOffsetMm.toFixed(0)
                        }
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1
                            border.color: penDisplayOffsetField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    Text {
                        text: "mm"
                        color: Theme.muted
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    AppButton {
                        height: 30
                        text: "적용"
                        onClicked: {
                            Backend.penDisplayOffsetMm = Number(penDisplayOffsetField.text)
                            penDisplayOffsetField.text = Backend.penDisplayOffsetMm.toFixed(0)
                        }
                    }
                    Text {
                        text: "TopView 표시 전용 · 서버 경로에는 영향 없음"
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Connections {
                        target: Backend
                        function onPenDisplayOffsetChanged() {
                            if (!penDisplayOffsetField.activeFocus)
                                penDisplayOffsetField.text = Backend.penDisplayOffsetMm.toFixed(0)
                        }
                    }
                }

                Text {
                    width: parent.width
                    text: "프로젝트 서버 기준값: 도장 폭 60 mm · 중심-펜 거리 172 mm. "
                          + "Qt와 서버는 이 값을 자동 동기화하지 않으므로 서버 설정을 바꾸면 여기에도 같은 값을 입력하세요."
                    color: (Math.abs(Backend.strokeWidthMm - 60) < 0.01
                            && Math.abs(Backend.penDisplayOffsetMm - 172) < 0.01)
                           ? Theme.muted : Theme.warn
                    font.pixelSize: 10
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }

                // ── 로봇 속도 ──────────────────────────────────────────
                // 도색 속도와 이동 속도는 다르다. 칠할 때는 도료가 고르게 깔려야
                // 해서 느리고, 도형 사이를 지나갈 때는 빠르게 가도 된다.
                Row {
                    width: parent.width
                    spacing: 8
                    component SpeedBox: Row {
                        id: sb
                        property string label
                        property string unit
                        property real value
                        property real step: 0.01
                        signal apply(real v)
                        spacing: 4
                        Text {
                            text: sb.label
                            color: Theme.sub
                            font.pixelSize: 12
                            font.family: Theme.fontFamily
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        TextField {
                            id: f
                            width: 60; height: 30
                            color: Theme.text
                            leftPadding: 8
                            selectByMouse: true
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            font.family: Theme.fontFamily
                            font.pixelSize: 12
                            text: sb.step < 1 ? sb.value.toFixed(2) : sb.value.toFixed(0)
                            onAccepted: sb.apply(Number(text))
                            onActiveFocusChanged: if (!activeFocus) sb.apply(Number(text))
                            background: Rectangle {
                                radius: 4; color: Theme.panel
                                border.width: 1
                                border.color: f.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                        Text {
                            text: sb.unit
                            color: Theme.muted
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                    SpeedBox {
                        label: "이동 속도"; unit: "m/s"
                        value: Backend.travelSpeedMps
                        onApply: (v) => Backend.travelSpeedMps = v
                    }
                    SpeedBox {
                        label: "도색 속도"; unit: "m/s"
                        value: Backend.paintSpeedMps
                        onApply: (v) => Backend.paintSpeedMps = v
                    }
                    SpeedBox {
                        label: "회전 속도"; unit: "°/s"; step: 1
                        value: Backend.turnSpeedDps
                        onApply: (v) => Backend.turnSpeedDps = v
                    }
                }
                Text {
                    width: parent.width
                    text: "⚠️ 이 속도는 화면 미리보기 재생에만 씁니다. "
                          + "프로토콜에 속도 항목이 없어 로봇으로는 전송되지 않고, "
                          + "실제 주행 속도는 로봇 펌웨어 고정값입니다."
                          + "\n예상 소요 시간은 로봇 사양의 직진·회전·ARC·노즐 시간을 기준으로 계산합니다. "
                          + "카메라 보정과 통신 대기는 제외되어 실제 작업은 더 걸릴 수 있습니다."
                          + "\n이 도면 예상 소요: " + Backend.planTimeText
                    color: Theme.muted
                    font.pixelSize: 10
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }

                // ⚠️ 여기 있던 "펜 오프셋" 입력란은 지웠다.
                //    BLUEPRINT.pen_offset_m 이 2026-07-28 프로토콜에서 폐지되고
                //    펜 오프셋 보정이 로봇 전담이 됐다 — 값을 받아둬도 쓰이는 곳이
                //    한 군데도 없어서 "설정했는데 왜 안 바뀌지"만 만들던 칸이다.

                Row {
                    width: parent.width
                    spacing: 8
                    Text {
                        text: "카메라 IP"
                        color: Theme.sub
                        font.pixelSize: 12
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    TextField {
                        id: camIpField
                        width: 150; height: 30
                        readOnly: true
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        placeholderText: "192.168.0.13"
                        placeholderTextColor: Theme.muted
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        Component.onCompleted: text = Backend.camIp
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1
                            border.color: camIpField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    AppButton {
                        height: 30
                        text: "서버에 저장"
                        ToolTip.visible: hovered
                        ToolTip.text: "운영 카메라 192.168.0.13을 서버 계정에도 저장합니다."
                        onClicked: Backend.applyCamIp(Backend.camIp)
                    }
                }

                Rectangle { width: parent.width; height: 1; color: Theme.stroke }

                // ── 4채널 중계 (PNM-C16083RVQ) ──────────────────────────
                Text {
                    text: "4채널 중계 주소"
                    color: Theme.sub
                    font.pixelSize: 12
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "MediaMTX 중계를 쓸 때만 서버 베이스 주소를 입력합니다. "
                        + "카메라에 직접 연결할 때는 비워두고 위의 카메라 IP만 적용하세요."
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
                Row {
                    width: parent.width
                    spacing: 8
                    TextField {
                        id: relayField
                        width: parent.width - 200
                        height: 30
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        placeholderText: "rtsp://192.168.0.2:8554   (카메라 직결이면 비움)"
                        placeholderTextColor: Theme.muted
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        Component.onCompleted: text = Backend.relayBase
                        onAccepted: Backend.setRelayBase(text)
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1
                            border.color: relayField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    AppButton {
                        height: 30
                        text: "적용"
                        ToolTip.visible: hovered
                        ToolTip.text: "채널 URL 은 규칙으로 조립합니다:\n"
                                    + "  메인 <주소>/ch1 … /ch4   (작업 화면)\n"
                                    + "  서브 <주소>/ch1s … /ch4s (2x2 미리보기)\n"
                                    + "Server/relay/README.md 의 경로와 같아야 합니다."
                        onClicked: Backend.setRelayBase(relayField.text)
                    }
                    AppButton {
                        height: 30
                        text: "비우기"
                        visible: Backend.relayBase.length > 0
                        onClicked: { relayField.text = ""; Backend.setRelayBase("") }
                    }
                }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    // 중계일 수도 카메라 직결일 수도 있어서 백엔드가 문장을 만들어 준다
                    // (relayBase 만 찍으면 직결일 때 빈칸이라 설정 누락처럼 보인다).
                    text: "현재 4채널 모드 · 채널 " + Backend.channelCount + "개 · "
                        + Backend.streamSourceText
                    color: Theme.accentDim
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
            }

            // ── 캘리브 탭 ──
            Column {
                width: parent.width
                spacing: 8
                visible: settingsPopup.tabIndex === 1

                Rectangle {
                    width: parent.width
                    height: robotCalibCol.implicitHeight + 28
                    radius: Theme.radius
                    color: Theme.panel
                    border.width: 1
                    border.color: Theme.stroke
                    Column {
                        id: robotCalibCol
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 14
                        spacing: 10
                        Item {
                            width: parent.width
                            height: 20
                            Text {
                                anchors.left: parent.left
                                anchors.verticalCenter: parent.verticalCenter
                                text: "로봇 주행 호모그래피"
                                color: Theme.text
                                font.pixelSize: 14
                                font.bold: true
                                font.family: Theme.fontFamily
                            }
                            Text {
                                anchors.right: parent.right
                                anchors.verticalCenter: parent.verticalCenter
                                text: Backend.calibratedChannels.indexOf(settingsPopup.calibrationChannel) >= 0
                                    ? "기존 보정 있음" : "보정 필요"
                                color: Backend.calibratedChannels.indexOf(settingsPopup.calibrationChannel) >= 0
                                    ? Theme.success : Theme.warn
                                font.pixelSize: 11
                                font.bold: true
                                font.family: Theme.fontFamily
                            }
                        }
                        Text {
                            width: parent.width
                            text: "채널을 선택해 시작만 요청합니다. 로봇 이동과 영상 계산은 서버·로봇·CCTV가 수행합니다."
                            color: Theme.sub
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            wrapMode: Text.WordWrap
                        }
                        // 이 채널이 undistort 를 선언해 놓고 K/D 를 안 보낸 상태인지.
                        // 배너가 다른 통지에 덮여도 여기서 채널별로 계속 확인된다.
                        Text {
                            width: parent.width
                            visible: Backend.lensDataMissingChannels.length > 0
                            text: "⚠️ 왜곡 보정 데이터(K/D) 없음: CH"
                                + Backend.lensDataMissingChannels.join(", CH")
                                + " — undistort 기준이라고 하는데 K/D 가 없어 좌표 오차가 커질 수 있습니다."
                            color: Theme.warn
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            wrapMode: Text.WordWrap
                        }
                        Row {
                            spacing: 6
                            Repeater {
                                model: Backend.channelCount
                                delegate: AppButton {
                                    required property int index
                                    width: 58
                                    text: "CH" + (index + 1)
                                    accent: settingsPopup.calibrationChannel === index + 1
                                    onClicked: settingsPopup.calibrationChannel = index + 1
                                }
                            }
                        }
                        // ── 오도메트리 주행 캘리브레이션 입력 (서버 요청서 20260813 §5-1) ──
                        Row {
                            spacing: 8
                            AppButton {
                                width: 96
                                text: "주행 캘리"
                                accent: settingsPopup.calibOdometry
                                onClicked: settingsPopup.calibOdometry = true
                            }
                            AppButton {
                                width: 96
                                text: "정적 앵커"
                                accent: !settingsPopup.calibOdometry
                                onClicked: settingsPopup.calibOdometry = false
                            }
                        }
                        Row {
                            spacing: 6
                            visible: settingsPopup.calibOdometry
                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                text: "가로"; color: Theme.sub
                                font.pixelSize: 11; font.family: Theme.fontFamily
                            }
                            TextField {
                                id: odoWidthField
                                width: 64; height: 28
                                // 🔴 validator 를 쓰지 않는다. validator 는 범위를 벗어난
                                //    입력의 editingFinished 를 막아 "화면 1200 / 전송 90"
                                //    이라는 조용한 불일치를 만든다. 여기서는 보이는 글자를
                                //    그대로 시작 버튼이 검증·전송한다.
                                text: String(Backend.odoWidthCm)
                                color: Theme.text
                                leftPadding: 6
                                font.pixelSize: 11; font.family: Theme.fontFamily
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                selectByMouse: true
                                // 유효할 때만 저장값을 갱신한다 (잘못된 값이 다음 세션으로
                                // 넘어가지 않게). 화면 글자는 사용자가 친 그대로 남는다.
                                onEditingFinished: if (Backend.odoSizeTextValid(text))
                                                       Backend.odoWidthCm = Number(text)
                                background: Rectangle {
                                    radius: 4; color: Theme.surface
                                    border.width: 1
                                    border.color: !Backend.odoSizeTextValid(odoWidthField.text)
                                                  ? Theme.danger
                                                  : (parent.activeFocus ? Theme.accent : Theme.stroke)
                                }
                            }
                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                text: "cm  세로"; color: Theme.sub
                                font.pixelSize: 11; font.family: Theme.fontFamily
                            }
                            TextField {
                                id: odoHeightField
                                width: 64; height: 28
                                text: String(Backend.odoHeightCm)
                                color: Theme.text
                                leftPadding: 6
                                font.pixelSize: 11; font.family: Theme.fontFamily
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                selectByMouse: true
                                onEditingFinished: if (Backend.odoSizeTextValid(text))
                                                       Backend.odoHeightCm = Number(text)
                                background: Rectangle {
                                    radius: 4; color: Theme.surface
                                    border.width: 1
                                    border.color: !Backend.odoSizeTextValid(odoHeightField.text)
                                                  ? Theme.danger
                                                  : (parent.activeFocus ? Theme.accent : Theme.stroke)
                                }
                            }
                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                text: "cm (각 2~1000cm)"; color: Theme.sub
                                font.pixelSize: 11; font.family: Theme.fontFamily
                            }
                        }
                        // 화면에 보이는 값이 그대로 전송된다. 보낼 수 없는 값이면
                        // 시작 버튼을 막고 이유를 여기서 먼저 말한다.
                        Text {
                            width: parent.width
                            visible: settingsPopup.calibOdometry
                                     && !(Backend.odoSizeTextValid(odoWidthField.text)
                                          && Backend.odoSizeTextValid(odoHeightField.text))
                            text: Backend.odoSizeRangeHint()
                            color: Theme.danger
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            wrapMode: Text.WordWrap
                        }
                        Row {
                            spacing: 8
                            visible: settingsPopup.calibOdometry
                            AppButton {
                                width: 110
                                text: "반시계(좌회전)"
                                accent: Backend.odoCcw
                                onClicked: Backend.odoCcw = true
                            }
                            AppButton {
                                width: 110
                                text: "시계(우회전)"
                                accent: !Backend.odoCcw
                                onClicked: Backend.odoCcw = false
                            }
                        }
                        // 🔴 사각형의 원점·방향이 로봇의 현재 위치·자세다 (요청서 §5-2).
                        Rectangle {
                            width: parent.width
                            height: odoPlaceText.implicitHeight + 18
                            radius: 5
                            visible: settingsPopup.calibOdometry
                            color: Theme.warnSoft
                            border.width: 1
                            border.color: Theme.warn
                            Text {
                                id: odoPlaceText
                                anchors.fill: parent
                                anchors.margins: 9
                                text: "시작 버튼을 누르는 순간의 로봇 중심이 사각형의 첫 꼭짓점입니다.\n"
                                    + (Backend.odoCcw
                                       ? "반시계: 왼쪽 아래에서 오른쪽으로 출발합니다. "
                                       : "시계: 왼쪽 위에서 오른쪽으로 출발합니다. ")
                                    + "로봇 앞면(주황색)을 출발 방향에 맞추고, 사각형 전체가 "
                                    + "카메라 화면 안에 들어오게 배치하세요."
                                color: Theme.text
                                font.pixelSize: 11
                                font.family: Theme.fontFamily
                                wrapMode: Text.WordWrap
                            }
                        }
                        Rectangle {
                            width: parent.width
                            height: safetyStartText.implicitHeight + 18
                            radius: 5
                            color: Theme.warnSoft
                            border.width: 1
                            border.color: Theme.warn
                            Text {
                                id: safetyStartText
                                anchors.fill: parent
                                anchors.margins: 9
                                text: settingsPopup.calibOdometry
                                    ? "⚠️ 주행 캘리는 로봇이 실제로 사각형을 돕니다 (2~4분). 사람과 장애물을 치우고 시작하세요."
                                    : "정적 앵커 방식에서는 로봇이 움직이지 않습니다. 계산이 끝날 때까지 기준 마커를 움직이지 마세요."
                                color: Theme.text
                                font.pixelSize: 11
                                font.family: Theme.fontFamily
                                wrapMode: Text.WordWrap
                            }
                        }
                        Row {
                            spacing: 8
                            AppButton {
                                accent: true
                                text: "CH" + settingsPopup.calibrationChannel
                                    + (settingsPopup.calibOdometry ? " 주행 캘리 시작" : " 호모그래피 시작")
                                enabled: !Backend.testMode && Backend.serverConnected
                                    && Backend.robotOnline && Backend.cctvOnline
                                    && !Backend.jobActive && !Backend.drawing
                                    && !Backend.homographyPending
                                    // 주행 방식일 때만 입력값이 조건이다. 정적 앵커는
                                    // 사각형 입력을 쓰지 않으므로 영향을 받지 않는다.
                                    && (!settingsPopup.calibOdometry
                                        || (Backend.odoSizeTextValid(odoWidthField.text)
                                            && Backend.odoSizeTextValid(odoHeightField.text)))
                                onClicked: {
                                    // 🔴 입력칸에 **보이는 글자**를 그대로 넘긴다.
                                    //    Backend 가 파싱·검증까지 하므로 화면 값과
                                    //    CALIB_START 값이 갈라질 수 없다.
                                    const ok = settingsPopup.calibOdometry
                                        ? Backend.startOdometryHomographyFromText(
                                              settingsPopup.calibrationChannel,
                                              odoWidthField.text, odoHeightField.text,
                                              Backend.odoCcw)
                                        : Backend.startHomography(settingsPopup.calibrationChannel, false)
                                    if (ok)
                                        settingsPopup.close()
                                }
                            }
                            Text {
                                anchors.verticalCenter: parent.verticalCenter
                                text: Backend.testMode ? "서버 로그인 필요"
                                    : (!Backend.serverConnected ? "서버 연결 필요"
                                    : (!Backend.robotOnline || !Backend.cctvOnline
                                       ? "로봇·CCTV 연결 필요" : ""))
                                color: Theme.warn
                                font.pixelSize: 10
                                font.family: Theme.fontFamily
                            }
                        }
                    }
                }

                AppButton {
                    text: settingsPopup.manualCalibrationExpanded
                        ? "고급 수동 보정 접기" : "고급 수동 보정 펼치기"
                    onClicked: settingsPopup.manualCalibrationExpanded = !settingsPopup.manualCalibrationExpanded
                }

                Column {
                    width: parent.width
                    spacing: 8
                    visible: settingsPopup.manualCalibrationExpanded

                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "가장자리 앵커 4점(RTSP 픽셀 + 월드 mm)으로 TopView를 펼칩니다.\n"
                        + "아래 1~4는 그냥 입력 순번입니다 — 마커 ID가 아닙니다. 서로 다른 마커 4개면 되고, "
                        + "네 귀퉁이를 쓰는 게 가장 정확합니다 (16마커 배치라면 ID 0·3·12·15).\n"
                        + "좌표계: 좌하단 (0,0) · X→오른쪽 · Y→위. 캔버스 크기는 월드 mm에 맞추며 600×600 고정이 아닙니다.\n"
                        + "CCTV가 준 캘리브레이션 JSON(K·D·H_floor)을 아래 칸에 붙여넣으면 앵커 없이도 적용됩니다."
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
                Rectangle {
                    width: parent.width
                    height: calibWarn.implicitHeight + 14
                    radius: 6
                    visible: Backend.calibMissing
                    color: Theme.warnSoft
                    border.width: 1
                    border.color: Theme.warn
                    Text {
                        id: calibWarn
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.margins: 8
                        anchors.verticalCenter: parent.verticalCenter
                        wrapMode: Text.WordWrap
                        text: "현재 사용할 수 있는 캘리브레이션이 없습니다.\n"
                            + "위에서 채널을 선택한 뒤 호모그래피 계산을 시작하세요."
                        color: Theme.text
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                    }
                }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "현재 · " + Backend.calibStatus
                    color: Theme.accent
                    font.pixelSize: 12
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                Text {
                    width: parent.width
                    text: "축척 · " + Backend.scaleText
                    color: Theme.sub
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }

                // 앵커 표
                Row {
                    width: parent.width
                    spacing: 4
                    // ⚠️ "ID" 가 아니라 입력 순번이다. 예전 헤더가 ID 라서 마커 ID 0~3 을
                    //    넣어야 하는 것처럼 보였는데, 실제로는 아무 마커 4개나 쓰면 된다.
                    Text { width: 28; text: "점"; color: Theme.muted; font.pixelSize: 10; font.family: Theme.fontFamily }
                    Text { width: 72; text: "px X"; color: Theme.muted; font.pixelSize: 10; font.family: Theme.fontFamily }
                    Text { width: 72; text: "px Y"; color: Theme.muted; font.pixelSize: 10; font.family: Theme.fontFamily }
                    Text { width: 64; text: "mm X"; color: Theme.muted; font.pixelSize: 10; font.family: Theme.fontFamily }
                    Text { width: 64; text: "mm Y"; color: Theme.muted; font.pixelSize: 10; font.family: Theme.fontFamily }
                }
                Repeater {
                    id: anchorRows
                    model: ListModel {
                        ListElement { aid: "1"; vpx: "938.27"; vpy: "731.0"; vmx: "60"; vmy: "60" }
                        ListElement { aid: "2"; vpx: "1114.0"; vpy: "597.5"; vmx: "540"; vmy: "60" }
                        ListElement { aid: "3"; vpx: "733.32"; vpy: "492.75"; vmx: "540"; vmy: "840" }
                        ListElement { aid: "4"; vpx: "534.0"; vpy: "576.75"; vmx: "60"; vmy: "840" }
                    }
                    delegate: Row {
                        id: anchorRow
                        width: 528
                        spacing: 4
                        readonly property string px: fPx.text
                        readonly property string py: fPy.text
                        readonly property string mx: fMx.text
                        readonly property string my: fMy.text
                        function setVals(a, b, c, d) {
                            fPx.text = a; fPy.text = b; fMx.text = c; fMy.text = d
                        }
                        Text {
                            width: 28; height: 28
                            text: aid
                            color: Theme.sub
                            font.pixelSize: 12
                            font.bold: true
                            font.family: Theme.fontFamily
                            verticalAlignment: Text.AlignVCenter
                        }
                        TextField {
                            id: fPx; width: 72; height: 28; text: vpx
                            color: Theme.text; leftPadding: 4
                            font.pixelSize: 11; font.family: Theme.fontFamily
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            background: Rectangle {
                                radius: 4; color: Theme.panel
                                border.width: 1; border.color: fPx.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                        TextField {
                            id: fPy; width: 72; height: 28; text: vpy
                            color: Theme.text; leftPadding: 4
                            font.pixelSize: 11; font.family: Theme.fontFamily
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            background: Rectangle {
                                radius: 4; color: Theme.panel
                                border.width: 1; border.color: fPy.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                        TextField {
                            id: fMx; width: 64; height: 28; text: vmx
                            color: Theme.text; leftPadding: 4
                            font.pixelSize: 11; font.family: Theme.fontFamily
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            background: Rectangle {
                                radius: 4; color: Theme.panel
                                border.width: 1; border.color: fMx.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                        TextField {
                            id: fMy; width: 64; height: 28; text: vmy
                            color: Theme.text; leftPadding: 4
                            font.pixelSize: 11; font.family: Theme.fontFamily
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                            background: Rectangle {
                                radius: 4; color: Theme.panel
                                border.width: 1; border.color: fMy.activeFocus ? Theme.accent : Theme.stroke
                            }
                        }
                    }
                }

                Row {
                    spacing: 8
                    width: parent.width
                    Text {
                        text: "캔버스 mm"
                        color: Theme.sub
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    TextField {
                        id: canvasWField
                        width: 70; height: 28
                        text: "600"
                        color: Theme.text
                        leftPadding: 6
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        placeholderText: "W"
                        placeholderTextColor: Theme.muted
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1; border.color: parent.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    Text { text: "×"; color: Theme.muted; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: canvasHField
                        width: 70; height: 28
                        text: "900"
                        color: Theme.text
                        leftPadding: 6
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        placeholderText: "H"
                        placeholderTextColor: Theme.muted
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1; border.color: parent.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    Text {
                        text: "(비우면 앵커+여유 자동)"
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }

                Text {
                    text: "선택 · 서버 H JSON (교차검증용)"
                    color: Theme.sub
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
                ScrollView {
                    width: parent.width
                    height: 56
                    clip: true
                    TextArea {
                        id: calibJsonField
                        width: parent.width
                        wrapMode: TextArea.Wrap
                        color: Theme.text
                        font.family: "Consolas"
                        font.pixelSize: 10
                        placeholderText: '{"H":[[...],[...],[...]]}  — 없어도 앵커 4점이면 계산됩니다'
                        placeholderTextColor: Theme.muted
                        background: Rectangle {
                            radius: 6
                            color: Theme.panel
                            border.width: 1
                            border.color: calibJsonField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                }

                Text {
                    id: calibResult
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: ""
                    color: Theme.sub
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    visible: text.length > 0
                }

                Row {
                    spacing: 6
                    AppButton {
                        accent: true
                        text: "앵커로 TopView 펼치기"
                        onClicked: calibResult.text = settingsPopup.applyCalib()
                    }
                    AppButton {
                        text: "내 앵커로 채우기"
                        onClicked: settingsPopup.fillDefaultAnchors()
                    }
                    AppButton {
                        text: "전체 지우기"
                        onClicked: settingsPopup.clearAnchors()
                    }
                }
                }
            }

            // ── 서버 탭 ──
            Column {
                width: parent.width
                spacing: 10
                visible: settingsPopup.tabIndex === 2

                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "중앙 서버(TLS, JSON Lines). role=QT 로 접속해 BLUEPRINT/CMD를 보내고 "
                        + "POSE·STATUS·PEERS·H_MATRIX·DRAW_FAIL을 받습니다."
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
                Row {
                    spacing: 8
                    Text {
                        text: "주소"
                        color: Theme.sub
                        font.pixelSize: 12
                        font.family: Theme.fontFamily
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    TextField {
                        id: hostField
                        width: 180; height: 30
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        Component.onCompleted: text = Backend.serverAddress.split(":")[0]
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1; border.color: hostField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    Text { text: ":"; color: Theme.muted; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: portField
                        width: 70; height: 30
                        color: Theme.text
                        leftPadding: 8
                        selectByMouse: true
                        inputMethodHints: Qt.ImhDigitsOnly
                        font.family: Theme.fontFamily
                        font.pixelSize: 12
                        Component.onCompleted: text = Backend.serverAddress.split(":")[1]
                        background: Rectangle {
                            radius: 4; color: Theme.panel
                            border.width: 1; border.color: portField.activeFocus ? Theme.accent : Theme.stroke
                        }
                    }
                    AppButton {
                        height: 30
                        accent: true
                        text: "적용 + 재연결"
                        onClicked: Backend.setServerAddress(hostField.text, Number(portField.text))
                    }
                }
                KvRow { k: "현재 연결"; v: Backend.serverLabel + " · " + Backend.serverAddress }
                KvRow { k: "카메라 IP (서버 등록)"; v: Backend.camIp.length ? Backend.camIp : "—" }
                KvRow { k: "관리자 창"; v: Backend.adminConsoleUrl }
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: "QT는 선택 채널에 CALIB_START만 요청합니다. 로봇 이동과 계산은 "
                        + "서버·로봇·CCTV가 수행하고, QT는 H_MATRIX 결과를 적용합니다."
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
            }

            Row {
                spacing: 6
                layoutDirection: Qt.RightToLeft
                width: parent.width
                AppButton { accent: true; text: "닫기"; onClicked: settingsPopup.close() }
            }
          }
        }

        function applyFilters() {
            Backend.setVideoFilters(fBright.value, fContrast.value, fSharpen.value, fSat.value)
        }

        function resetVideoFilters() {
            fBright.value = 45
            fContrast.value = 8
            fSharpen.value = 0
            fSat.value = 0
            applyFilters()
        }

        function fillDefaultAnchors() {
            const vals = [
                ["938.27", "731.0", "60", "60"],
                ["1114.0", "597.5", "540", "60"],
                ["733.32", "492.75", "540", "840"],
                ["534.0", "576.75", "60", "840"]
            ]
            for (let i = 0; i < anchorRows.count && i < vals.length; ++i) {
                const row = anchorRows.itemAt(i)
                if (row && row.setVals)
                    row.setVals(vals[i][0], vals[i][1], vals[i][2], vals[i][3])
            }
            canvasWField.text = "600"
            canvasHField.text = "900"
        }

        // 앵커 칸을 하나씩 지우기 번거로워서 한 번에 비운다.
        // JSON 칸은 남긴다 — 보통 앵커만 다시 찍고 H 는 그대로 쓰기 때문이다.
        function clearAnchors() {
            for (let i = 0; i < anchorRows.count; ++i) {
                const row = anchorRows.itemAt(i)
                if (row && row.setVals)
                    row.setVals("", "", "", "")
            }
            calibResult.text = "앵커를 모두 비웠습니다."
        }

        function applyCalib() {
            try {
                let obj = {}
                const raw = (calibJsonField.text || "").trim()
                if (raw.length)
                    obj = JSON.parse(raw)

                // CCTV 번들은 {"calib":{...}} 로 한 겹 감싸 온다. H 유무는 안을 봐야 안다.
                const inner = (obj && obj.calib) ? obj.calib : obj
                const hasH = !!(inner && (inner.H || inner.H_floor))

                const corners = []
                let anchorErr = ""
                for (let i = 0; i < anchorRows.count; ++i) {
                    const row = anchorRows.itemAt(i)
                    if (!row) continue
                    const px = Number(row.px), py = Number(row.py)
                    const mx = Number(row.mx), my = Number(row.my)
                    if (![px, py, mx, my].every(isFinite)) {
                        anchorErr = "앵커 id" + i + " 숫자 형식을 확인하세요."
                        continue
                    }
                    corners.push({ id: "id" + i, px: [px, py], mm: [mx, my] })
                }

                // H 행렬 하나로도 좌표계가 결정되므로, H 가 있으면 앵커는 없어도 된다.
                // (CCTV 가 준 번들을 그대로 붙여넣는 경우가 이것이다)
                if (corners.length < 4 && !hasH)
                    return anchorErr || "앵커 4점 또는 H 행렬이 필요합니다."

                obj.unit = "mm"
                if (corners.length >= 4)
                    obj.corners = corners
                obj.origin_mm = [0, 0]

                const cw = Number(canvasWField.text)
                const ch = Number(canvasHField.text)
                if (isFinite(cw) && isFinite(ch) && cw >= 10 && ch >= 10)
                    obj.canvas_mm = [cw, ch]

                return Backend.applyCalibJson(JSON.stringify(obj))
            } catch (e) {
                return "JSON 형식 오류: " + e
            }
        }
    }
}

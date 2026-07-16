import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Dialogs
import QtCore
import RoadPainter

ApplicationWindow {
    id: root

    width: 1460
    height: 920
    minimumWidth: 1220
    minimumHeight: 760
    visible: true
    title: "Road-Painter — 관제 클라이언트 (Mock)"
    color: Theme.bg0

    // "select" | "draw" | "measure"
    property string mode: "select"
    property string statusMessage: "준비 완료 — 좌측 도구로 경로를 작도하세요. (작도: 클릭 추가 · 우클릭 완료)"
    property real mockFps: 29.9
    property real pathLength: 0
    property string exportSpace: "pixel"

    function setStatus(msg) { statusMessage = msg }

    function setMode(m) {
        mode = m
        if (m === "select")
            setStatus("선택/편집 모드 — 정점 드래그 이동 · 정점 우클릭 삭제 · 박스 드래그 전체 이동 · 우하단 핸들 크기 조절")
        else if (m === "draw")
            setStatus("작도 모드 — 화면 클릭으로 정점 추가, 우클릭으로 작도 완료")
        else if (m === "measure")
            setStatus("측정 모드 — 커서 위치의 영상 좌표를 표시합니다. 클릭 시 마커 고정")
    }

    function exportOptions() {
        return {
            closed: pathModel.closed,
            coordinateSpace: exportSpace,
            workAreaWidth: 8.0,
            workAreaHeight: 4.5
        }
    }

    function doExport(fileUrl) {
        if (pathModel.exportToFile(fileUrl, exportOptions())) {
            toast.show("경로 JSON 내보내기 완료", "ok")
            setStatus("저장 완료: " + fileUrl)
        } else {
            toast.show("저장 실패: " + pathModel.lastError, "error")
        }
    }

    // ── 백엔드 ───────────────────────────────────────
    PathModel {
        id: pathModel
        imageWidth: 1280
        imageHeight: 720
    }

    PresetLibrary { id: presets }

    FilterSettings { id: filters }

    RobotSimulator {
        id: robot
        onFinished: {
            toast.show("도장 작업 완료 — 라인 품질을 확인하세요", "ok")
            setStatus("로봇 도장 작업 완료 (결과 검사 대기)")
        }
    }

    Connections {
        target: pathModel
        function onPathChanged() { root.pathLength = pathModel.totalLength() }
    }

    Timer {
        interval: 700
        running: true
        repeat: true
        onTriggered: root.mockFps = 29.2 + Math.random() * 1.5
    }

    Shortcut { sequence: "Escape"; onActivated: root.setMode("select") }
    Shortcut { sequence: "Ctrl+Z"; onActivated: pathModel.removeLast() }
    Shortcut { sequence: "Ctrl+E"; onActivated: exportFileDialog.open() }

    // ── 헤더 ─────────────────────────────────────────
    header: Rectangle {
        height: 56
        color: Theme.bg1

        Rectangle {
            anchors.bottom: parent.bottom
            width: parent.width
            height: 1
            color: Theme.strokeSoft
        }

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: Theme.sp4
            anchors.rightMargin: Theme.sp4
            spacing: Theme.sp4

            Rectangle {
                width: 32; height: 32; radius: 8
                color: Theme.accent
                Text {
                    anchors.centerIn: parent
                    text: "R"
                    color: "#1c1206"
                    font.pixelSize: 18
                    font.bold: true
                }
            }

            ColumnLayout {
                spacing: 1
                Text {
                    text: "ROAD-PAINTER"
                    color: Theme.textHi
                    font.pixelSize: Theme.fontLg
                    font.bold: true
                    font.letterSpacing: 1.2
                }
                Text {
                    text: "인프라 비전 측위 관제 클라이언트 · QML Mock"
                    color: Theme.textLow
                    font.pixelSize: Theme.fontXs
                }
            }

            Item { Layout.fillWidth: true }

            Rectangle {
                width: modeChipText.width + 24
                height: 26
                radius: 13
                color: Qt.alpha(Theme.accent, 0.13)
                border.color: Qt.alpha(Theme.accent, 0.5)
                Text {
                    id: modeChipText
                    anchors.centerIn: parent
                    text: root.mode === "draw" ? "경로 작도 모드"
                        : root.mode === "measure" ? "좌표 측정 모드" : "선택/편집 모드"
                    color: Theme.accent
                    font.pixelSize: Theme.fontXs
                    font.bold: true
                }
            }

            Row {
                spacing: 6
                Rectangle {
                    width: 8; height: 8; radius: 4
                    anchors.verticalCenter: parent.verticalCenter
                    color: Theme.ok
                }
                Text {
                    text: "TLS 연결됨 · MOCK"
                    color: Theme.textMid
                    font.pixelSize: Theme.fontXs
                    font.family: Theme.mono
                }
            }

            Text {
                id: headerClock
                color: Theme.textMid
                font.pixelSize: Theme.fontSm
                font.family: Theme.mono
                Timer {
                    interval: 1000
                    running: true
                    repeat: true
                    triggeredOnStart: true
                    onTriggered: headerClock.text = Qt.formatDateTime(new Date(), "hh:mm:ss")
                }
            }
        }
    }

    // ── 본문 ─────────────────────────────────────────
    RowLayout {
        anchors.fill: parent
        anchors.margins: Theme.sp3
        spacing: Theme.sp3

        // 좌측 툴바
        Rectangle {
            Layout.preferredWidth: 64
            Layout.fillHeight: true
            color: Theme.bg1
            radius: Theme.radius
            border.color: Theme.strokeSoft

            Column {
                anchors.top: parent.top
                anchors.topMargin: Theme.sp2
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 6

                SideToolButton {
                    iconName: "cursor"; label: "선택"
                    checked: root.mode === "select"
                    onClicked: root.setMode("select")
                }
                SideToolButton {
                    iconName: "pen"; label: "작도"
                    checked: root.mode === "draw"
                    onClicked: root.setMode("draw")
                }
                SideToolButton {
                    iconName: "measure"; label: "측정"
                    checked: root.mode === "measure"
                    onClicked: root.setMode("measure")
                }

                Rectangle {
                    width: 40; height: 1
                    anchors.horizontalCenter: parent.horizontalCenter
                    color: Theme.strokeSoft
                }

                SideToolButton {
                    iconName: "preset"; label: "프리셋"
                    onClicked: presetDialog.open()
                }
                SideToolButton {
                    iconName: "trash"; label: "지우기"
                    danger: true
                    onClicked: {
                        if (pathModel.count === 0) {
                            toast.show("지울 경로가 없습니다", "warn")
                            return
                        }
                        pathModel.clear()
                        toast.show("경로를 모두 지웠습니다", "info")
                    }
                }
            }
        }

        // 중앙: 뷰포트 + 상태 스트립
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.sp2

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#040507"
                radius: Theme.radius
                border.color: Theme.strokeSoft
                clip: true

                VideoViewport {
                    id: viewport
                    anchors.fill: parent
                    anchors.margins: 6
                    mode: root.mode
                    pathModel: pathModel
                    robot: robot
                    filters: filters
                    onDrawFinished: {
                        root.setMode("select")
                        saveDialog.pointCount = pathModel.count
                        saveDialog.open()
                    }
                }

                Toast {
                    id: toast
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 22
                }
            }

            // 상태 스트립
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 32
                color: Theme.bg1
                radius: Theme.radiusSmall
                border.color: Theme.strokeSoft

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: Theme.sp3
                    anchors.rightMargin: Theme.sp3
                    spacing: Theme.sp4

                    Row {
                        spacing: 6
                        Rectangle {
                            width: 8; height: 8; radius: 4
                            anchors.verticalCenter: parent.verticalCenter
                            color: Theme.ok
                            SequentialAnimation on opacity {
                                loops: Animation.Infinite
                                NumberAnimation { to: 0.35; duration: 900 }
                                NumberAnimation { to: 1.0; duration: 900 }
                            }
                        }
                        Text {
                            text: "LIVE(MOCK) · 1280×720 · " + root.mockFps.toFixed(1) + "fps"
                            color: Theme.textMid
                            font.pixelSize: Theme.fontXs
                            font.family: Theme.mono
                        }
                    }

                    Text {
                        text: viewport.imageCursor.x >= 0
                              ? "커서 X " + viewport.imageCursor.x + " · Y " + viewport.imageCursor.y
                              : "커서 — 영상 밖"
                        color: Theme.textMid
                        font.pixelSize: Theme.fontXs
                        font.family: Theme.mono
                    }

                    Text {
                        text: "정점 " + pathModel.count + " · 길이 " + Math.round(root.pathLength) + "px"
                        color: Theme.textMid
                        font.pixelSize: Theme.fontXs
                        font.family: Theme.mono
                    }

                    Text {
                        Layout.fillWidth: true
                        text: root.statusMessage
                        color: Theme.textLow
                        font.pixelSize: Theme.fontXs
                        elide: Text.ElideRight
                        horizontalAlignment: Text.AlignRight
                    }
                }
            }
        }

        // 우측 패널
        Flickable {
            Layout.preferredWidth: 324
            Layout.fillHeight: true
            contentHeight: panelColumn.implicitHeight
            clip: true
            boundsBehavior: Flickable.StopAtBounds

            ColumnLayout {
                id: panelColumn
                width: parent.width
                spacing: Theme.sp3

                PanelSection {
                    title: "채널 관제 (4CH)"
                    Layout.fillWidth: true

                    ChannelStrip {
                        width: parent.width
                        liveItem: viewport.videoContentItem
                    }
                }

                PanelSection {
                    title: "비디오 필터"
                    Layout.fillWidth: true

                    LabeledSlider {
                        width: parent.width
                        label: "밝기"
                        from: -100; to: 100
                        value: filters.brightness
                        onMoved: (v) => filters.brightness = Math.round(v)
                    }
                    LabeledSlider {
                        width: parent.width
                        label: "대비"
                        from: -100; to: 100
                        value: filters.contrast
                        onMoved: (v) => filters.contrast = Math.round(v)
                    }
                    LabeledSlider {
                        width: parent.width
                        label: "채도"
                        from: -100; to: 100
                        value: filters.saturation
                        onMoved: (v) => filters.saturation = Math.round(v)
                    }
                    LabeledSlider {
                        width: parent.width
                        label: "선명도"
                        from: 0; to: 100
                        value: filters.sharpen
                        onMoved: (v) => filters.sharpen = Math.round(v)
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        AppSwitch {
                            id: graySwitch
                            text: "그레이스케일"
                            checked: filters.grayscale
                            onToggled: filters.grayscale = checked
                        }
                        Item { width: parent.width - graySwitch.width - resetBtn.width - 2 * Theme.sp2; height: 1 }
                        AppButton {
                            id: resetBtn
                            text: "초기화"
                            onClicked: {
                                filters.reset()
                                toast.show("필터를 초기화했습니다", "info")
                            }
                        }
                    }
                }

                PanelSection {
                    title: "경로 데이터"
                    Layout.fillWidth: true

                    AppSwitch {
                        text: "폐곡선 (시작점으로 복귀)"
                        checked: pathModel.closed
                        onToggled: pathModel.closed = checked
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        Text {
                            text: "내보내기 좌표계"
                            color: Theme.textMid
                            font.pixelSize: Theme.fontSm
                            anchors.verticalCenter: parent.verticalCenter
                        }

                        Repeater {
                            model: [
                                { key: "pixel", label: "픽셀" },
                                { key: "meter", label: "미터(모의)" }
                            ]
                            delegate: Rectangle {
                                required property var modelData
                                readonly property bool active: root.exportSpace === modelData.key
                                width: chipText.width + 20
                                height: 24
                                radius: 12
                                color: active ? Qt.alpha(Theme.accent, 0.15) : Theme.bg2
                                border.color: active ? Qt.alpha(Theme.accent, 0.6) : Theme.stroke

                                Text {
                                    id: chipText
                                    anchors.centerIn: parent
                                    text: parent.modelData.label
                                    color: parent.active ? Theme.accent : Theme.textMid
                                    font.pixelSize: Theme.fontXs
                                }
                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: root.exportSpace = parent.modelData.key
                                }
                            }
                        }
                    }

                    Text {
                        visible: root.exportSpace === "meter"
                        width: parent.width
                        text: "※ 미터 좌표는 8.0 × 4.5 m 작업 구역 모의 캘리브레이션(선형 매핑)입니다. 실제 시스템에서는 호모그래피 변환이 적용됩니다."
                        color: Theme.textLow
                        font.pixelSize: Theme.fontXs
                        wrapMode: Text.WordWrap
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        AppButton {
                            width: (parent.width - Theme.sp2) / 2
                            text: "마지막 점 삭제"
                            enabled: pathModel.count > 0
                            onClicked: pathModel.removeLast()
                        }
                        AppButton {
                            width: (parent.width - Theme.sp2) / 2
                            text: "전체 지우기"
                            variant: "danger"
                            enabled: pathModel.count > 0
                            onClicked: {
                                pathModel.clear()
                                toast.show("경로를 모두 지웠습니다", "info")
                            }
                        }
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        AppButton {
                            width: (parent.width - Theme.sp2) / 2
                            text: "JSON 내보내기"
                            variant: "primary"
                            enabled: pathModel.count >= 2
                            onClicked: exportFileDialog.open()
                        }
                        AppButton {
                            width: (parent.width - Theme.sp2) / 2
                            text: "JSON 불러오기"
                            onClicked: importFileDialog.open()
                        }
                    }
                }

                PanelSection {
                    title: "로봇 이동 시뮬레이션"
                    Layout.fillWidth: true

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        Rectangle {
                            width: stateChipText.width + 20
                            height: 24
                            radius: 12
                            color: robot.state === "painting" ? Qt.alpha(Theme.accent, 0.15)
                                 : robot.state === "done" ? Qt.alpha(Theme.ok, 0.15)
                                 : robot.state === "transit" ? Qt.alpha(Theme.warn, 0.15)
                                 : Theme.bg2
                            border.color: robot.state === "painting" ? Theme.accent
                                        : robot.state === "done" ? Theme.ok
                                        : robot.state === "transit" ? Theme.warn
                                        : Theme.stroke

                            Text {
                                id: stateChipText
                                anchors.centerIn: parent
                                text: robot.stateText
                                color: robot.state === "painting" ? Theme.accent
                                     : robot.state === "done" ? Theme.ok
                                     : robot.state === "transit" ? Theme.warn
                                     : Theme.textMid
                                font.pixelSize: Theme.fontXs
                            }
                        }

                        Text {
                            anchors.verticalCenter: parent.verticalCenter
                            text: Math.round(robot.progress * 100) + "%"
                            color: Theme.textHi
                            font.pixelSize: Theme.fontSm
                            font.family: Theme.mono
                        }
                    }

                    Rectangle {
                        width: parent.width
                        height: 6
                        radius: 3
                        color: Theme.bg3

                        Rectangle {
                            width: parent.width * robot.progress
                            height: parent.height
                            radius: 3
                            color: robot.state === "done" ? Theme.ok : Theme.accent
                            Behavior on width { NumberAnimation { duration: 80 } }
                        }
                    }

                    LabeledSlider {
                        width: parent.width
                        label: "주행 속도"
                        from: 40; to: 300
                        value: robot.speed
                        suffix: " px/s"
                        onMoved: (v) => robot.speed = v
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        Text {
                            text: "도료 색상"
                            color: Theme.textMid
                            font.pixelSize: Theme.fontSm
                            anchors.verticalCenter: parent.verticalCenter
                        }

                        Repeater {
                            model: ["#f2f4f6", "#ffd23f", "#ff7a1a"]
                            delegate: Rectangle {
                                required property string modelData
                                width: 22; height: 22; radius: 11
                                color: modelData
                                border.color: Qt.colorEqual(viewport.trailColor, modelData)
                                              ? Theme.accent : Theme.stroke
                                border.width: Qt.colorEqual(viewport.trailColor, modelData) ? 2.5 : 1

                                MouseArea {
                                    anchors.fill: parent
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: viewport.trailColor = parent.modelData
                                }
                            }
                        }
                    }

                    Row {
                        width: parent.width
                        spacing: Theme.sp2

                        AppButton {
                            width: (parent.width - 2 * Theme.sp2) / 3
                            text: robot.state === "paused" ? "재개" : "시작"
                            variant: "primary"
                            enabled: !robot.running
                            onClicked: {
                                if (robot.state === "paused") {
                                    robot.start()
                                    setStatus("로봇 주행 재개")
                                    return
                                }
                                if (pathModel.count < 2) {
                                    toast.show("경로 정점이 2개 이상 필요합니다", "warn")
                                    return
                                }
                                robot.setPath(pathModel.points(), pathModel.closed)
                                robot.start()
                                setStatus("로봇 주행 시작 — 시작점으로 이동 중")
                            }
                        }
                        AppButton {
                            width: (parent.width - 2 * Theme.sp2) / 3
                            text: "일시정지"
                            enabled: robot.running
                            onClicked: {
                                robot.pause()
                                setStatus("로봇 일시정지 (Fail-Safe: 펜 상태 유지)")
                            }
                        }
                        AppButton {
                            width: (parent.width - 2 * Theme.sp2) / 3
                            text: "리셋"
                            onClicked: {
                                robot.reset()
                                setStatus("로봇을 대기 지점으로 복귀시키고 도장 궤적을 지웠습니다")
                            }
                        }
                    }
                }

                Item { Layout.preferredHeight: 2 }
            }
        }
    }

    // ── 다이얼로그 ───────────────────────────────────
    PresetDialog {
        id: presetDialog
        library: presets
        onApplied: (presetId) => {
            pathModel.setPoints(presets.generate(presetId, 730, 420, 150))
            pathModel.closed = (presetId === "rectangle" || presetId === "triangle"
                                || presetId === "hexagon")
            root.setMode("select")
            toast.show("프리셋 로드 완료 — 드래그로 이동, 핸들로 크기 조절", "ok")
        }
    }

    PathSaveDialog {
        id: saveDialog
        onSaveTemp: {
            if (pathModel.exportToFile(pathModel.tempFileUrl(), root.exportOptions())) {
                toast.show("임시 저장 완료 (path_temp.json)", "ok")
                setStatus("임시 저장: " + pathModel.tempFileUrl())
            } else {
                toast.show("임시 저장 실패: " + pathModel.lastError, "error")
            }
        }
        onExportRequested: exportFileDialog.open()
    }

    FileDialog {
        id: exportFileDialog
        title: "경로 JSON 내보내기"
        fileMode: FileDialog.SaveFile
        nameFilters: ["JSON 파일 (*.json)"]
        defaultSuffix: "json"
        currentFolder: StandardPaths.writableLocation(StandardPaths.DocumentsLocation)
        onAccepted: root.doExport(selectedFile)
    }

    FileDialog {
        id: importFileDialog
        title: "경로 JSON 불러오기"
        fileMode: FileDialog.OpenFile
        nameFilters: ["JSON 파일 (*.json)"]
        currentFolder: StandardPaths.writableLocation(StandardPaths.DocumentsLocation)
        onAccepted: {
            if (pathModel.importFromFile(selectedFile)) {
                toast.show("경로 불러오기 완료 — 정점 " + pathModel.count + "개", "ok")
                root.setMode("select")
            } else {
                toast.show("불러오기 실패: " + pathModel.lastError, "error")
            }
        }
    }
}

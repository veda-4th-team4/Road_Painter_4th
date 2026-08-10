import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Rectangle {
    id: pane
    property string title: ""
    property bool topRole: false
    property bool highlight: false
    property alias view: vv
    // LIVE 는 "우리 화면에 실제 영상이 들어오는 중"일 때만. PEERS 의 cctv 는
    // 서버-카메라 접속 여부라서, 그것만 보고 LIVE 를 켜면 오프라인 캔버스인데도 LIVE 로 보인다.
    property bool liveBadge: !topRole && Backend.cctvOnline && !Backend.offlineCanvas

    color: "#1A1D21"
    radius: Theme.radius
    border.width: highlight ? 2 : 1
    border.color: highlight ? Theme.accent : Theme.stroke
    clip: true

    VideoView {
        id: vv
        anchors.fill: parent
        topView: pane.topRole
        Component.onCompleted: Backend.registerView(vv, pane.topRole)
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 10
        z: 2
        radius: 4
        color: "#CCFFFFFF"
        border.color: Theme.stroke
        border.width: 1
        width: titleRow.implicitWidth + 14
        height: 24
        Row {
            id: titleRow
            anchors.centerIn: parent
            spacing: 6
            Rectangle {
                width: 7; height: 7; radius: 4
                anchors.verticalCenter: parent.verticalCenter
                color: pane.liveBadge ? Theme.success
                     : (pane.highlight ? Theme.accent : Theme.muted)
            }
            Text {
                text: pane.liveBadge ? (pane.title + "  LIVE") : pane.title
                color: Theme.text
                font.pixelSize: 11
                font.bold: true
                font.family: Theme.fontFamily
            }
        }
    }

    Rectangle {
        visible: Backend.jobActive || Backend.phase === "done"
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 10
        z: 2
        radius: 4
        color: "#F2FFFFFF"
        border.color: Theme.accent
        border.width: 1
        width: pctLabel.implicitWidth + 16
        height: 26
        Text {
            id: pctLabel
            anchors.centerIn: parent
            text: Backend.jobPercent + "%"
            color: Theme.accentDim
            font.bold: true
            font.pixelSize: 12
            font.family: Theme.fontFamily
        }
    }

    // 축척 눈금자 — 끌어서 옮길 수 있고, 놓으면 가까운 모서리로 붙는다
    ScaleRuler {
        visible: pane.topRole && vv.screenPxPerMm > 0
        pxPerMm: vv.screenPxPerMm
        mmPerPx: vv.mmPerPx
    }

    // ── 확대/축소 바 (좌하단) ────────────────────────────────────────
    // 휠을 모르는 사람도 마우스만으로 배율을 조절할 수 있게 슬라이더를 둔다.
    // "영역 확대"를 켜면 드래그한 사각형에 맞춰 확대된다 (CAD 의 윈도우 줌).
    Rectangle {
        id: zoomBar
        visible: pane.topRole
        z: 5
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.leftMargin: 10
        anchors.bottomMargin: 10
        width: zoomRow.implicitWidth + 16
        height: 30
        radius: 15
        color: "#F2FFFFFF"
        border.color: zoomHover.hovered || vv.zoomTool ? Theme.accent : Theme.stroke
        border.width: 1
        opacity: zoomHover.hovered || vv.zoomTool ? 1.0 : 0.82
        Behavior on opacity { NumberAnimation { duration: 120 } }
        HoverHandler { id: zoomHover }

        // 25% ~ 1200% 를 슬라이더 0~1 에 로그로 펼친다 (중간이 100% 근처)
        readonly property real span: Math.log(1200 / 25)
        function toSlider(pct) { return Math.log(Math.max(25, pct) / 25) / span }
        function toPct(t) { return Math.round(25 * Math.exp(t * span)) }

        Row {
            id: zoomRow
            anchors.centerIn: parent
            spacing: 6

            AppButton {
                width: 24; height: 22
                text: "−"
                anchors.verticalCenter: parent.verticalCenter
                onClicked: vv.zoomBy(1 / 1.25)
            }
            Slider {
                id: zoomSlider
                width: 108
                height: 22
                anchors.verticalCenter: parent.verticalCenter
                from: 0; to: 1
                onMoved: vv.setZoomPercent(zoomBar.toPct(value))
                // ⚠️ value 를 바인딩으로 두면 안 된다. 사용자가 손잡이를 끄는 순간
                // Slider 가 value 에 직접 써서 바인딩이 끊기고, 그 뒤로는
                // [100%] 를 눌러 뷰가 초기화돼도 손잡이가 제자리로 안 돌아온다.
                // 배율이 바뀔 때마다 명시적으로 따라오게 한다.
                Component.onCompleted: value = zoomBar.toSlider(vv.zoomPercent)
                Connections {
                    target: vv
                    function onScaleChanged() {
                        if (!zoomSlider.pressed)
                            zoomSlider.value = zoomBar.toSlider(vv.zoomPercent)
                    }
                }
                background: Rectangle {
                    x: 0; y: zoomSlider.height / 2 - 2
                    width: zoomSlider.width
                    height: 4
                    radius: 2
                    color: Theme.stroke
                    Rectangle {
                        width: zoomSlider.visualPosition * parent.width
                        height: parent.height
                        radius: 2
                        color: Theme.accent
                    }
                }
                handle: Rectangle {
                    x: zoomSlider.visualPosition * (zoomSlider.width - width)
                    y: zoomSlider.height / 2 - height / 2
                    width: 12; height: 12; radius: 6
                    color: zoomSlider.pressed ? Theme.accent : "#FFFFFF"
                    border.color: Theme.accent
                    border.width: 1
                }
                ToolTip.visible: hovered || pressed
                ToolTip.text: "끌어서 확대/축소 (" + vv.zoomPercent + "%)"
            }
            AppButton {
                width: 24; height: 22
                text: "+"
                anchors.verticalCenter: parent.verticalCenter
                onClicked: vv.zoomBy(1.25)
            }
            Rectangle {
                width: 1; height: 16; color: Theme.stroke
                anchors.verticalCenter: parent.verticalCenter
            }
            AppButton {
                width: 26; height: 22
                text: "⛶"
                accent: vv.zoomTool
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "영역 확대 — 켠 뒤 화면을 드래그하면 그 사각형에 맞춰 확대됩니다"
                onClicked: vv.zoomTool = !vv.zoomTool
            }
            AppButton {
                width: 26; height: 22
                text: "⤢"
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "선택(없으면 도형 전체)에 맞춰 확대"
                onClicked: vv.zoomToSelection()
            }
            AppButton {
                width: 40; height: 22
                text: "100%"
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "전체 보기 (Ctrl+0)"
                onClicked: vv.fitView()
            }
        }
    }

    // 영역 확대 모드일 때 안내 — 켜둔 걸 잊고 점을 못 찍는 일이 없도록
    Rectangle {
        visible: pane.topRole && vv.zoomTool
        z: 5
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 46
        width: zoomHint.implicitWidth + 20
        height: 26
        radius: 13
        color: "#E6FFD600"
        Text {
            id: zoomHint
            anchors.centerIn: parent
            text: "영역 확대 모드 — 드래그해서 확대, 끄려면 ⛶ 다시 클릭"
            color: "#1A1D21"
            font.pixelSize: 11
            font.bold: true
            font.family: Theme.fontFamily
        }
    }

    DropArea {
        anchors.fill: vv
        enabled: pane.topRole && !Backend.jobActive
        keys: ["preset-shape"]
        onDropped: function(drop) {
            if (Backend.phase === "done")
                Backend.clearMission()
            if (drop.source && drop.source.shape !== undefined)
                vv.addPresetAt(drop.source.shape, drop.x, drop.y)
            drop.accept()
        }
    }

    // ── 변 길이(mm) 직접 입력 ────────────────────────────────────────
    // TopView 위의 mm 라벨을 클릭하면 그 자리에서 수치를 고칠 수 있다.
    Connections {
        target: vv
        enabled: pane.topRole
        function onEdgeEditRequested(index, mm, vx, vy) {
            if (Backend.jobActive) return
            vertexEditor.close()
            edgeEditor.edgeIndex = index
            edgeField.text = mm.toFixed(mm >= 100 ? 0 : 1)
            edgeEditor.x = Math.max(6, Math.min(pane.width - edgeEditor.width - 6,
                                                vx - edgeEditor.width / 2))
            // 라벨 위가 좁으면 아래쪽에 띄운다
            const above = vy - edgeEditor.height - 12
            edgeEditor.y = (above >= 6) ? above
                         : Math.min(pane.height - edgeEditor.height - 6, vy + 14)
            edgeEditor.visible = true
            edgeField.selectAll()
            edgeField.forceActiveFocus()
        }
        // 꼭짓점 회전각 배지를 클릭하면 각도를 직접 고칠 수 있다.
        function onTurnEditRequested(index, deg, vx, vy) {
            if (Backend.jobActive) return
            edgeEditor.close()
            vertexEditor.close()
            turnEditor.vertexIndex = index
            turnField.text = deg.toFixed(1)
            turnEditor.x = Math.max(6, Math.min(pane.width - turnEditor.width - 6,
                                                vx - turnEditor.width / 2))
            const above = vy - turnEditor.height - 12
            turnEditor.y = (above >= 6) ? above
                         : Math.min(pane.height - turnEditor.height - 6, vy + 14)
            turnEditor.visible = true
            turnField.selectAll()
            turnField.forceActiveFocus()
        }
        // 점을 더블클릭하면 (x, y) mm 가 뜨고 그 자리에서 고칠 수 있다.
        function onVertexEditRequested(index, xMm, yMm, vx, vy) {
            if (Backend.jobActive) return
            edgeEditor.close()
            turnEditor.close()
            vertexEditor.vertexIndex = index
            vxField.text = xMm.toFixed(1)
            vyField.text = yMm.toFixed(1)
            vertexEditor.x = Math.max(6, Math.min(pane.width - vertexEditor.width - 6,
                                                  vx - vertexEditor.width / 2))
            const vAbove = vy - vertexEditor.height - 14
            vertexEditor.y = (vAbove >= 6) ? vAbove
                           : Math.min(pane.height - vertexEditor.height - 6, vy + 16)
            vertexEditor.visible = true
            vxField.selectAll()
            vxField.forceActiveFocus()
        }
        function onPathChanged() {
            if (!vertexEditor.activeEdit)
                vertexEditor.visible = false
            if (!edgeEditor.activeEdit)
                edgeEditor.visible = false
            if (!turnEditor.activeEdit)
                turnEditor.visible = false
        }
        function onShowLabelsChanged() {
            if (!vv.showLabels) {
                edgeEditor.close()
                turnEditor.close()
            }
        }
    }

    // ── 꼭짓점 회전각(°) 직접 입력 ───────────────────────────────────
    // 프로토콜 TURN 과 같은 부호 규약: 양수 = 좌회전, 음수 = 우회전.
    Rectangle {
        id: turnEditor
        z: 6
        visible: false
        width: turnCol.implicitWidth + 16
        height: turnCol.implicitHeight + 14
        radius: 6
        color: Theme.surface
        border.color: Theme.accent
        border.width: 1
        property int vertexIndex: -1
        property bool activeEdit: false

        function apply() {
            const v = Number(turnField.text)
            if (!isFinite(v)) { close(); return }
            activeEdit = true
            vv.setTurnAngleAt(vertexIndex, v)
            activeEdit = false
            close()
        }
        function quick(v) {
            turnField.text = String(v)
            apply()
        }
        function close() {
            visible = false
            vertexIndex = -1
        }

        Column {
            id: turnCol
            anchors.centerIn: parent
            spacing: 6

            Row {
                spacing: 6
                Text {
                    text: "꼭짓점 " + (turnEditor.vertexIndex + 1) + " 회전"
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    anchors.verticalCenter: parent.verticalCenter
                }
                TextField {
                    id: turnField
                    width: 66
                    height: 28
                    color: Theme.text
                    leftPadding: 6
                    font.pixelSize: 12
                    font.family: Theme.fontFamily
                    horizontalAlignment: Text.AlignRight
                    inputMethodHints: Qt.ImhFormattedNumbersOnly
                    anchors.verticalCenter: parent.verticalCenter
                    validator: DoubleValidator {
                        bottom: -179; top: 179; decimals: 1
                        notation: DoubleValidator.StandardNotation
                    }
                    background: Rectangle {
                        radius: 4
                        color: Theme.panel
                        border.width: 1
                        border.color: turnField.activeFocus ? Theme.accent : Theme.stroke
                    }
                    onAccepted: turnEditor.apply()
                    Keys.onEscapePressed: turnEditor.close()
                }
                Text {
                    text: "°"
                    color: Theme.sub
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    anchors.verticalCenter: parent.verticalCenter
                }
                AppButton {
                    width: 44; height: 28
                    accent: true
                    text: "적용"
                    anchors.verticalCenter: parent.verticalCenter
                    onClicked: turnEditor.apply()
                }
                AppButton {
                    width: 28; height: 28
                    text: "✕"
                    anchors.verticalCenter: parent.verticalCenter
                    onClicked: turnEditor.close()
                }
            }

            // 자주 쓰는 각도 — 도로 표시는 대부분 직각/45°
            Row {
                spacing: 4
                Text {
                    text: "빠른 각"
                    color: Theme.muted
                    font.pixelSize: 10
                    font.family: Theme.fontFamily
                    anchors.verticalCenter: parent.verticalCenter
                }
                Repeater {
                    model: [90, 45, 30, -30, -45, -90]
                    AppButton {
                        required property int modelData
                        width: 42; height: 24
                        text: (modelData > 0 ? "↺" : "↻") + Math.abs(modelData)
                        onClicked: turnEditor.quick(modelData)
                    }
                }
            }
            Text {
                text: "양수 = 좌회전 · 음수 = 우회전 (뒤쪽 경로가 함께 돕니다)"
                color: Theme.muted
                font.pixelSize: 10
                font.family: Theme.fontFamily
            }
        }
    }

    Rectangle {
        id: edgeEditor
        z: 6
        visible: false
        width: editRow.implicitWidth + 16
        height: 40
        radius: 6
        color: Theme.surface
        border.color: Theme.accent
        border.width: 1
        property int edgeIndex: -1
        property bool activeEdit: false

        function apply() {
            const v = Number(edgeField.text)
            if (!isFinite(v) || v <= 0) { close(); return }
            activeEdit = true
            vv.setEdgeLengthMm(edgeIndex, v)
            activeEdit = false
            close()
        }
        function close() {
            visible = false
            edgeIndex = -1
        }

        Row {
            id: editRow
            anchors.centerIn: parent
            spacing: 6
            Text {
                text: "변 " + (edgeEditor.edgeIndex + 1)
                color: Theme.muted
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            TextField {
                id: edgeField
                width: 78
                height: 28
                color: Theme.text
                leftPadding: 6
                font.pixelSize: 12
                font.family: Theme.fontFamily
                horizontalAlignment: Text.AlignRight
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                anchors.verticalCenter: parent.verticalCenter
                validator: DoubleValidator { bottom: 0.1; top: 1000000; decimals: 2; notation: DoubleValidator.StandardNotation }
                background: Rectangle {
                    radius: 4
                    color: Theme.panel
                    border.width: 1
                    border.color: edgeField.activeFocus ? Theme.accent : Theme.stroke
                }
                onAccepted: edgeEditor.apply()
                Keys.onEscapePressed: edgeEditor.close()
            }
            Text {
                text: "mm"
                color: Theme.sub
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            AppButton {
                width: 44; height: 28
                accent: true
                text: "적용"
                anchors.verticalCenter: parent.verticalCenter
                onClicked: edgeEditor.apply()
            }
            AppButton {
                width: 28; height: 28
                text: "✕"
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "취소 (Esc)"
                onClicked: edgeEditor.close()
            }
        }
    }

    // ── 꼭짓점 좌표(mm) 직접 입력 ────────────────────────────────────
    // 점을 더블클릭하면 뜬다. 눈대중으로 찍은 점을 도면 수치로 바로잡는 용도.
    Rectangle {
        id: vertexEditor
        z: 7
        visible: false
        width: vertexRow.implicitWidth + 16
        height: 40
        radius: 6
        color: Theme.surface
        border.color: Theme.accent
        border.width: 1
        property int vertexIndex: -1
        property bool activeEdit: false

        function apply() {
            const px = Number(vxField.text)
            const py = Number(vyField.text)
            if (!isFinite(px) || !isFinite(py)) { close(); return }
            activeEdit = true
            vv.setVertexWorldMm(vertexIndex, px, py)
            activeEdit = false
            close()
        }
        function close() {
            visible = false
            vertexIndex = -1
        }

        Row {
            id: vertexRow
            anchors.centerIn: parent
            spacing: 5

            Text {
                text: "점 " + (vertexEditor.vertexIndex + 1)
                color: Theme.muted
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            Text {
                text: "("
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            TextField {
                id: vxField
                width: 76
                height: 28
                color: Theme.text
                leftPadding: 6
                font.pixelSize: 12
                font.family: Theme.fontFamily
                horizontalAlignment: Text.AlignRight
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                anchors.verticalCenter: parent.verticalCenter
                validator: DoubleValidator { bottom: -1000000; top: 1000000; decimals: 1; notation: DoubleValidator.StandardNotation }
                background: Rectangle {
                    radius: 4
                    color: Theme.panel
                    border.width: 1
                    border.color: vxField.activeFocus ? Theme.accent : Theme.stroke
                }
                onAccepted: vertexEditor.apply()
                Keys.onEscapePressed: vertexEditor.close()
                KeyNavigation.tab: vyField
            }
            Text {
                text: ","
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            TextField {
                id: vyField
                width: 76
                height: 28
                color: Theme.text
                leftPadding: 6
                font.pixelSize: 12
                font.family: Theme.fontFamily
                horizontalAlignment: Text.AlignRight
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                anchors.verticalCenter: parent.verticalCenter
                validator: DoubleValidator { bottom: -1000000; top: 1000000; decimals: 1; notation: DoubleValidator.StandardNotation }
                background: Rectangle {
                    radius: 4
                    color: Theme.panel
                    border.width: 1
                    border.color: vyField.activeFocus ? Theme.accent : Theme.stroke
                }
                onAccepted: vertexEditor.apply()
                Keys.onEscapePressed: vertexEditor.close()
            }
            Text {
                text: ") mm"
                color: Theme.sub
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            AppButton {
                width: 44; height: 28
                accent: true
                text: "적용"
                anchors.verticalCenter: parent.verticalCenter
                onClicked: vertexEditor.apply()
            }
            AppButton {
                width: 28; height: 28
                text: "✕"
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "취소 (Esc)"
                onClicked: vertexEditor.close()
            }
        }
    }

}

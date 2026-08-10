import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

// 4채널 미리보기 화면 (PNM-C16083RVQ).
//
// 흐름:  타일 클릭 → 하이라이트 + [작업하기] 활성화 → 누르면 그 채널 1개만
//        메인스트림 + 마커검출 + 기존 Qt 작업 화면.
//
// 여기서는 **마커를 검출하지 않는다.** 4채널에 ArUco 를 다 돌리면 CPU 가 녹고
// (1080p 기준 검출 한 번이 20~100ms), 미리보기에서는 마커가 필요하지도 않다.
Rectangle {
    id: grid
    color: Theme.bg

    readonly property int cols: 2
    readonly property int rows: Math.ceil(Backend.channelCount / cols)

    Column {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12
        visible: !Backend.homographyPending

        // ── 헤더 ────────────────────────────────────────────────────────
        Item {
            width: parent.width
            height: 34
            Column {
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                spacing: 2
                Text {
                    text: "채널 선택"
                    color: Theme.text
                    font.pixelSize: 17
                    font.bold: true
                    font.family: Theme.fontFamily
                }
                Text {
                    text: "작업할 채널을 고른 뒤 [작업하기]를 누르세요 · 미리보기는 마커를 검출하지 않습니다"
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
            }
            Row {
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                spacing: 8
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    // 중계/직결 어느 쪽이든 맞는 문장이 나온다 (Backend 가 만든다)
                    text: Backend.streamSourceText
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                }
                // 한 채널만 까맣거나 안 돌아올 때 쓰는 수동 탈출구.
                // 평소엔 필요 없다 — 끊기면 워커가 알아서 다시 붙는다.
                AppButton {
                    text: "새로고침"
                    ToolTip.visible: hovered
                    ToolTip.text: "미리보기 " + Backend.channelCount + "채널을 전부 다시 엽니다.\n"
                                + "다시 여는 데 몇 초 걸립니다 (채널당 약 1초, 순차)."
                    onClicked: Backend.refreshStreams()
                }
                AppButton {
                    accent: Backend.canStartChannelWork
                    enabled: Backend.canStartChannelWork
                    text: Backend.highlightedChannel > 0
                          ? "CH" + Backend.highlightedChannel + " 작업하기"
                          : "작업하기"
                    ToolTip.visible: hovered
                    ToolTip.text: Backend.highlightedChannel === 0
                        ? "먼저 채널을 선택하세요"
                        : (Backend.calibratedChannels.indexOf(Backend.highlightedChannel) >= 0
                           ? "CH" + Backend.highlightedChannel + " 메인스트림 + 마커검출로 전환합니다"
                           // 막지는 않는다 — 캘리 없이도 영상 확인과 수동 조작은 필요하다.
                           // 다만 그 상태로 그린 도면은 엉뚱한 곳에 칠해진다.
                           : "⚠ CH" + Backend.highlightedChannel + " 은 캘리브레이션이 없습니다.\n"
                             + "영상·수동 조작은 되지만 도면 좌표는 맞지 않습니다.")
                    onClicked: Backend.startChannelWork()
                }
            }
        }

        // ── 2x2 타일 ────────────────────────────────────────────────────
        Grid {
            id: tileGrid
            width: parent.width
            height: parent.height - 34 - 12
            columns: grid.cols
            spacing: 10

            Repeater {
                model: Backend.channelCount
                delegate: Rectangle {
                    id: cell
                    required property int index
                    readonly property int ch: index + 1
                    readonly property bool isSelected: Backend.highlightedChannel === cell.ch
                    // ⚠️ Backend.channelCalibrated(ch) 를 직접 부르면 안 된다 —
                    //    Q_INVOKABLE 은 바인딩 의존성이 안 잡혀서, 캘리브레이션이
                    //    나중에 들어와도 화면이 "없음"인 채로 남는다.
                    readonly property bool hasCalib:
                        Backend.calibratedChannels.indexOf(cell.ch) >= 0

                    width: (tileGrid.width - tileGrid.spacing * (grid.cols - 1)) / grid.cols
                    height: (tileGrid.height - tileGrid.spacing * (grid.rows - 1)) / grid.rows
                    color: "#14161A"
                    radius: Theme.radius
                    clip: true
                    // 선택 표시는 테두리로 준다 — 영상 위에 반투명 막을 씌우면
                    // 정작 봐야 할 현장이 어두워진다.
                    border.width: cell.isSelected ? 3 : 1
                    border.color: cell.isSelected ? Theme.accent : Theme.stroke

                    ChannelTile {
                        id: tile
                        anchors.fill: parent
                        anchors.margins: cell.border.width
                        Component.onCompleted: Backend.registerTile(tile, cell.ch)
                    }
                    Connections {
                        target: Backend
                        function onHomographyChanged() {
                            // 대기 화면의 단일 타일이 같은 채널 등록을 잠시 가져간다.
                            // 대기 종료 뒤 2x2 타일을 다시 등록해야 프레임이 돌아온다.
                            if (!Backend.homographyPending)
                                Backend.registerTile(tile, cell.ch)
                        }
                    }

                    // 채널 라벨 + 스트림 상태 점
                    Rectangle {
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.margins: 8
                        z: 2
                        radius: 4
                        color: "#CCFFFFFF"
                        border.color: Theme.stroke
                        border.width: 1
                        width: label.implicitWidth + 14
                        height: 24
                        Row {
                            id: label
                            anchors.centerIn: parent
                            spacing: 6
                            Rectangle {
                                width: 7; height: 7; radius: 4
                                anchors.verticalCenter: parent.verticalCenter
                                color: tile.failed ? Theme.danger
                                     : (tile.live ? Theme.success : Theme.muted)
                            }
                            Text {
                                text: "CH" + cell.ch
                                color: Theme.text
                                font.pixelSize: 11
                                font.bold: true
                                font.family: Theme.fontFamily
                            }
                        }
                    }

                    // 캘리브레이션이 없는 채널은 고르기 전에 알려준다. 눌러본 뒤에야
                    // "작업하기가 왜 안 되지"를 알게 하면 안 된다.
                    Rectangle {
                        visible: !cell.hasCalib
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 8
                        z: 2
                        radius: 4
                        color: Theme.warnSoft
                        border.color: Theme.warn
                        border.width: 1
                        width: warnLabel.implicitWidth + 14
                        height: 24
                        Text {
                            id: warnLabel
                            anchors.centerIn: parent
                            text: "캘리브레이션 없음"
                            color: Theme.warn
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                        }
                    }

                    // 더블클릭으로 바로 들어가기. 단일 클릭(하이라이트)은 ChannelTile 이
                    // 처리하므로 여기서는 press 를 흘려보내 두 경로가 안 싸우게 한다.
                    MouseArea {
                        anchors.fill: parent
                        acceptedButtons: Qt.LeftButton
                        propagateComposedEvents: true
                        onDoubleClicked: {
                            Backend.highlightChannel(cell.ch)
                            Backend.startChannelWork()
                        }
                        onPressed: function(mouse) { mouse.accepted = false }
                    }
                }
            }
        }
    }

    Item {
        id: homographyView
        anchors.fill: parent
        anchors.margins: 16
        visible: Backend.homographyPending

        Rectangle {
            id: selectedVideo
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: parent.width * 0.68
            radius: Theme.radius
            color: "#14161A"
            border.width: 2
            border.color: Theme.accent
            clip: true

            ChannelTile {
                id: homographyTile
                anchors.fill: parent
                anchors.margins: 2
                onVisibleChanged: {
                    if (visible && Backend.homographyChannel > 0)
                        Backend.registerTile(homographyTile, Backend.homographyChannel)
                }
            }
            Connections {
                target: Backend
                function onHomographyChanged() {
                    if (Backend.homographyPending && Backend.homographyChannel > 0)
                        Backend.registerTile(homographyTile, Backend.homographyChannel)
                }
            }

            Rectangle {
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.margins: 12
                width: liveLabel.implicitWidth + 18
                height: 28
                radius: 4
                color: "#E6FFFFFF"
                Text {
                    id: liveLabel
                    anchors.centerIn: parent
                    text: "CH" + Backend.homographyChannel + " 실시간 영상"
                    color: Theme.text
                    font.pixelSize: 12
                    font.bold: true
                    font.family: Theme.fontFamily
                }
            }
        }

        Rectangle {
            anchors.left: selectedVideo.right
            anchors.leftMargin: 14
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            radius: Theme.radius
            color: Theme.surface
            border.width: 1
            border.color: Theme.stroke

            Column {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.margins: 24
                spacing: 18

                BusyIndicator {
                    anchors.horizontalCenter: parent.horizontalCenter
                    running: Backend.homographyPending
                    width: 48
                    height: 48
                }
                Text {
                    width: parent.width
                    text: "CH" + Backend.homographyChannel + " 호모그래피 계산 중"
                    horizontalAlignment: Text.AlignHCenter
                    color: Theme.text
                    font.pixelSize: 20
                    font.bold: true
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }
                Text {
                    width: parent.width
                    text: Backend.homographyStatus
                    horizontalAlignment: Text.AlignHCenter
                    color: Theme.sub
                    font.pixelSize: 12
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }
                ProgressBar {
                    width: parent.width
                    visible: Backend.homographyProgress >= 0
                    from: 0
                    to: 1
                    value: Math.max(0, Backend.homographyProgress)
                }
                Rectangle {
                    width: parent.width
                    height: safetyText.implicitHeight + 24
                    radius: 6
                    color: Theme.warnSoft
                    border.width: 1
                    border.color: Theme.warn
                    Text {
                        id: safetyText
                        anchors.fill: parent
                        anchors.margins: 12
                        text: "로봇이 자동으로 이동할 수 있습니다.\n작업 영역에 들어가지 마세요."
                        horizontalAlignment: Text.AlignHCenter
                        color: Theme.text
                        font.pixelSize: 12
                        font.bold: true
                        font.family: Theme.fontFamily
                        wrapMode: Text.WordWrap
                    }
                }
                Text {
                    width: parent.width
                    text: "완료되면 새 보정값을 적용하고 이 채널의 작업 화면으로 자동 전환합니다."
                    horizontalAlignment: Text.AlignHCenter
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }
                AppButton {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: Backend.homographyCancelPending ? "중단 확인 대기 중" : "호모그래피 중단"
                    enabled: !Backend.homographyCancelPending
                    danger: true
                    onClicked: Backend.cancelHomography()
                }
            }
        }
    }
}

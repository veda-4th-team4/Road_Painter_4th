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

            // 대기 화면 오른쪽 패널. 위에서 아래로 "무엇을/어디까지/안전/다음 행동"
            // 순서로 읽히게 두고, 값이 늘고 줄어도 레이아웃이 흔들리지 않도록
            // 진행 영역 높이를 고정한다.
            Column {
                id: waitPanel
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                anchors.margins: 18
                spacing: 12

                // 서버 응답을 기다리는 단계 이름. 화면 문구를 한 곳에서 만든다.
                readonly property string phase: !Backend.homographyPending ? ""
                    : (Backend.homographyCancelPending ? "cancelling"
                    : (Backend.homographyCancelUnconfirmed ? "cancel_unconfirmed"
                    : (Backend.homographyOdometry
                        ? (Backend.homographyPhase === "" ? "requesting" : Backend.homographyPhase)
                        : "solving")))
                readonly property bool driving: phase === "driving"

                // ── 헤더: 채널 + 방식 ─────────────────────────────────────
                Row {
                    width: parent.width
                    spacing: 8
                    Text {
                        text: "CH" + Backend.homographyChannel
                        color: Theme.text
                        font.pixelSize: 20
                        font.bold: true
                        font.family: Theme.fontFamily
                    }
                    Rectangle {
                        anchors.verticalCenter: parent.verticalCenter
                        width: methodLabel.implicitWidth + 14
                        height: 22
                        radius: 4
                        color: Backend.homographyOdometry ? Theme.warnSoft : Theme.mutedSoft
                        border.width: 1
                        border.color: Backend.homographyOdometry ? Theme.warn : Theme.stroke
                        Text {
                            id: methodLabel
                            anchors.centerIn: parent
                            text: Backend.homographyOdometry ? "주행 캘리브레이션" : "정적 앵커 캘리브레이션"
                            color: Theme.text
                            font.pixelSize: 11
                            font.bold: true
                            font.family: Theme.fontFamily
                        }
                    }
                }

                // ── 현재 단계 ─────────────────────────────────────────────
                Row {
                    width: parent.width
                    spacing: 10
                    BusyIndicator {
                        anchors.verticalCenter: parent.verticalCenter
                        running: Backend.homographyPending
                        width: 24
                        height: 24
                    }
                    Column {
                        width: parent.width - 34
                        spacing: 2
                        Text {
                            width: parent.width
                            text: waitPanel.phase === "requesting" ? "서버 응답을 기다리는 중"
                                : waitPanel.phase === "driving" ? "로봇이 사각형을 주행하며 촬영 중"
                                : waitPanel.phase === "solving" ? "카메라가 호모그래피를 계산 중"
                                : waitPanel.phase === "cancelling" ? "중단 요청 — 정지 확인 대기"
                                : waitPanel.phase === "cancel_unconfirmed" ? "정지 확인을 받지 못함"
                                : "진행 중"
                            color: waitPanel.phase === "cancel_unconfirmed" ? Theme.warn : Theme.text
                            font.pixelSize: 13
                            font.bold: true
                            font.family: Theme.fontFamily
                            wrapMode: Text.WordWrap
                        }
                        Text {
                            width: parent.width
                            text: Backend.homographyStatus
                            color: Theme.sub
                            font.pixelSize: 11
                            font.family: Theme.fontFamily
                            wrapMode: Text.WordWrap
                            maximumLineCount: 3
                            elide: Text.ElideRight
                        }
                    }
                }

                // ── 정지점 진행 (주행 방식 전용) ──────────────────────────
                // 서버가 값을 주기 전에는 점만 비워 둔다 — 가짜 진행률은 만들지 않는다.
                Column {
                    width: parent.width
                    spacing: 6
                    visible: Backend.homographyOdometry
                    Row {
                        width: parent.width
                        spacing: 4
                        Repeater {
                            model: Math.max(Backend.homographyPointTotal, 9)
                            delegate: Rectangle {
                                required property int index
                                width: (waitPanel.width - 4 * (Math.max(Backend.homographyPointTotal, 9) - 1))
                                       / Math.max(Backend.homographyPointTotal, 9)
                                height: 6
                                radius: 3
                                color: index <= Backend.homographyPointIndex ? Theme.accent : Theme.stroke
                            }
                        }
                    }
                    Text {
                        width: parent.width
                        text: Backend.homographyPointIndex < 0
                            ? "정지점 0 / " + Math.max(Backend.homographyPointTotal, 9) + " 완료"
                            : ("정지점 " + (Backend.homographyPointIndex + 1) + " / "
                               + Math.max(Backend.homographyPointTotal, 9) + " 완료   ·   유효 "
                               + Math.max(0, Backend.homographyValidPoints) + "개")
                        color: Backend.homographyCaptureLag ? Theme.warn : Theme.sub
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                    }
                    Text {
                        width: parent.width
                        visible: Backend.homographyCaptureLag
                        text: "카메라가 로봇을 놓치고 있습니다. 유효 대응점이 6개 미만이면 실패합니다."
                        color: Theme.warn
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                        wrapMode: Text.WordWrap
                    }
                }

                // 정적 앵커는 서버 진행률이 있을 때만 막대를 보여준다.
                ProgressBar {
                    width: parent.width
                    visible: !Backend.homographyOdometry && Backend.homographyProgress >= 0
                    from: 0
                    to: 1
                    value: Math.max(0, Backend.homographyProgress)
                }

                // ── 안전 / 다음 행동 ──────────────────────────────────────
                Rectangle {
                    width: parent.width
                    height: safetyText.implicitHeight + 20
                    radius: 6
                    color: Backend.homographyOdometry ? Theme.warnSoft : Theme.mutedSoft
                    border.width: 1
                    border.color: Backend.homographyOdometry ? Theme.warn : Theme.stroke
                    Text {
                        id: safetyText
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.margins: 10
                        anchors.verticalCenter: parent.verticalCenter
                        text: Backend.homographyOdometry
                            ? "로봇이 지금 사각형을 주행합니다 (약 2~4분). 작업 영역과 사각형 안으로 들어가지 마세요."
                            : "로봇은 움직이지 않습니다. 카메라가 정지 상태에서 계산합니다."
                        color: Theme.text
                        font.pixelSize: 11
                        font.bold: Backend.homographyOdometry
                        font.family: Theme.fontFamily
                        wrapMode: Text.WordWrap
                    }
                }
                Text {
                    width: parent.width
                    text: "완료되면 새 보정값을 적용하고 이 채널의 작업 화면으로 전환합니다."
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }

                // ── 중단 ──────────────────────────────────────────────────
                // 취소는 서버가 로봇 정지를 확인한 뒤에야 확정된다. 확인이 오지
                // 않으면 "취소됨"이 아니라 확인 실패로 알리고 재시도를 허용한다.
                AppButton {
                    width: parent.width
                    text: Backend.homographyCancelPending ? "정지 확인 대기 중…"
                        : (Backend.homographyCancelUnconfirmed ? "중단 다시 요청" : "캘리브레이션 중단")
                    enabled: !Backend.homographyCancelPending
                    danger: true
                    onClicked: Backend.cancelHomography()
                }
                Text {
                    width: parent.width
                    visible: Backend.homographyCancelPending || Backend.homographyCancelUnconfirmed
                    text: Backend.homographyCancelPending
                        ? "로봇이 실제로 섰다는 서버 확인을 기다립니다."
                        : "서버 확인이 없습니다. 로봇 정지를 직접 확인하고, 필요하면 비상정지를 사용하세요."
                    color: Backend.homographyCancelUnconfirmed ? Theme.warn : Theme.muted
                    font.pixelSize: 10
                    font.family: Theme.fontFamily
                    wrapMode: Text.WordWrap
                }
            }
        }
    }
}

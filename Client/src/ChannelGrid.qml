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
                    text: Backend.relayBase
                    color: Theme.muted
                    font.pixelSize: 11
                    font.family: Theme.fontFamily
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
}

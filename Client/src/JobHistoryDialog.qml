import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

// 작업 이력 — 지금까지 그린 도면을 계획(before) → 결과(after) 로 보여주고,
// 그대로 다시 그리거나, 편집기로 불러와 고치거나, 이름을 바꾸거나 지운다.
Popup {
    id: dlg
    modal: true
    anchors.centerIn: Overlay.overlay
    width: Math.min(780, Overlay.overlay ? Overlay.overlay.width - 60 : 780)
    // 항목 수에 맞춰 줄어든다 — 두 건뿐인데 빈 칸이 넓게 남지 않도록
    height: Math.min(Math.max(220, 88 + Backend.historyCount * 116),
                     Overlay.overlay ? Overlay.overlay.height - 60 : 620)
    padding: 0

    property string renamingId: ""
    property string confirmDeleteId: ""

    onOpened: { renamingId = ""; confirmDeleteId = "" }

    background: Rectangle {
        radius: Theme.radius
        color: Theme.surface
        border.color: Theme.stroke
        border.width: 1
    }

    function statusColor(s) {
        if (s === "완료") return Theme.success
        if (s === "중단") return Theme.danger
        if (s === "진행") return Theme.accent
        return Theme.muted
    }

    Item {
        anchors.fill: parent
        anchors.margins: 16

        Row {
            id: head
            width: parent.width
            spacing: 8
            Text {
                text: "작업 이력"
                color: Theme.text
                font.pixelSize: 15
                font.bold: true
                font.family: Theme.fontFamily
                anchors.verticalCenter: parent.verticalCenter
            }
            Rectangle {
                width: cntTxt.implicitWidth + 14
                height: 20
                radius: 10
                color: Theme.elevated
                anchors.verticalCenter: parent.verticalCenter
                Text {
                    id: cntTxt
                    anchors.centerIn: parent
                    text: Backend.historyCount + "건"
                    color: Theme.sub
                    font.pixelSize: 10
                    font.bold: true
                    font.family: Theme.fontFamily
                }
            }
            Item { width: parent.width - 320; height: 1 }
            AppButton {
                height: 26
                text: "현재 도면 저장"
                enabled: Backend.hasPath
                anchors.verticalCenter: parent.verticalCenter
                ToolTip.visible: hovered
                ToolTip.text: "지금 작도 중인 도면을 전송하지 않고 이력에만 저장합니다"
                onClicked: Backend.saveCurrentDrawing("")
            }
            AppButton {
                height: 26
                text: "닫기"
                anchors.verticalCenter: parent.verticalCenter
                onClicked: dlg.close()
            }
        }

        Text {
            id: emptyHint
            anchors.centerIn: parent
            visible: Backend.historyCount === 0
            text: "아직 저장된 작업이 없습니다.\n경로를 전송하면 자동으로 이력에 남습니다."
            color: Theme.muted
            font.pixelSize: 12
            font.family: Theme.fontFamily
            horizontalAlignment: Text.AlignHCenter
        }

        ListView {
            id: list
            anchors.top: head.bottom
            anchors.topMargin: 12
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            clip: true
            spacing: 8
            visible: Backend.historyCount > 0
            model: Backend.jobHistory
            ScrollBar.vertical: ScrollBar {}

            delegate: Rectangle {
                id: card
                required property var modelData
                width: list.width - 12
                height: 108
                radius: Theme.radius
                color: Theme.panel
                border.color: cardHover.hovered ? Theme.accent : Theme.stroke
                border.width: 1
                HoverHandler { id: cardHover }

                // ── before → after 미리보기 ──
                Row {
                    id: thumbs
                    anchors.left: parent.left
                    anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 6

                    Column {
                        spacing: 2
                        Rectangle {
                            width: 96; height: 66
                            radius: 4
                            color: "#12151A"
                            border.color: Theme.stroke
                            border.width: 1
                            PathThumb {
                                anchors.fill: parent
                                anchors.margins: 2
                                paths: card.modelData.paths
                                closedFlags: card.modelData.closed
                                mode: "plan"
                            }
                        }
                        Text {
                            text: "계획"
                            color: Theme.muted
                            font.pixelSize: 9
                            font.family: Theme.fontFamily
                            width: 96
                            horizontalAlignment: Text.AlignHCenter
                        }
                    }

                    Text {
                        text: "→"
                        color: Theme.muted
                        font.pixelSize: 14
                        anchors.verticalCenter: parent.verticalCenter
                    }

                    Column {
                        spacing: 2
                        Rectangle {
                            width: 96; height: 66
                            radius: 4
                            color: "#12151A"
                            border.color: Theme.stroke
                            border.width: 1
                            PathThumb {
                                anchors.fill: parent
                                anchors.margins: 2
                                paths: card.modelData.paths
                                closedFlags: card.modelData.closed
                                mode: "result"
                                progress: card.modelData.progress
                            }
                        }
                        Text {
                            text: "결과 " + Math.round(card.modelData.progress * 100) + "%"
                            color: Theme.muted
                            font.pixelSize: 9
                            font.family: Theme.fontFamily
                            width: 96
                            horizontalAlignment: Text.AlignHCenter
                        }
                    }
                }

                // ── 정보 ──
                Column {
                    anchors.left: thumbs.right
                    anchors.leftMargin: 14
                    anchors.right: actions.left
                    anchors.rightMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 5

                    Row {
                        spacing: 6
                        width: parent.width
                        Text {
                            visible: dlg.renamingId !== card.modelData.id
                            text: card.modelData.name
                            color: Theme.text
                            font.pixelSize: 13
                            font.bold: true
                            font.family: Theme.fontFamily
                            elide: Text.ElideRight
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        TextField {
                            id: nameField
                            visible: dlg.renamingId === card.modelData.id
                            width: 180
                            height: 26
                            text: card.modelData.name
                            color: Theme.text
                            leftPadding: 6
                            font.pixelSize: 12
                            font.family: Theme.fontFamily
                            anchors.verticalCenter: parent.verticalCenter
                            background: Rectangle {
                                radius: 4
                                color: Theme.surface
                                border.width: 1
                                border.color: Theme.accent
                            }
                            onAccepted: {
                                Backend.renameJob(card.modelData.id, text)
                                dlg.renamingId = ""
                            }
                            Keys.onEscapePressed: dlg.renamingId = ""
                        }
                        Rectangle {
                            visible: dlg.renamingId !== card.modelData.id
                            width: stTxt.implicitWidth + 12
                            height: 17
                            radius: 8
                            color: Theme.elevated
                            anchors.verticalCenter: parent.verticalCenter
                            Text {
                                id: stTxt
                                anchors.centerIn: parent
                                text: card.modelData.status
                                color: dlg.statusColor(card.modelData.status)
                                font.pixelSize: 9
                                font.bold: true
                                font.family: Theme.fontFamily
                            }
                        }
                    }

                    Text {
                        text: card.modelData.created
                        color: Theme.muted
                        font.pixelSize: 10
                        font.family: Theme.fontFamily
                    }
                    Text {
                        text: "도형 " + card.modelData.shapes + "개 · 점 " + card.modelData.points
                              + "개 · 경로 " + card.modelData.lengthM.toFixed(2) + " m"
                        color: Theme.sub
                        font.pixelSize: 11
                        font.family: Theme.fontFamily
                    }
                }

                // ── 동작 ──
                Column {
                    id: actions
                    anchors.right: parent.right
                    anchors.rightMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 4

                    // "다시 그리기"는 없앴다 — 새 흐름에서는 도면을 올려도 로봇이
                    // 안 움직여서 버튼 이름이 실제 동작과 어긋난다.
                    // 불러온 뒤 [도면 전송] → [그림그리기 시작] 순서를 그대로 밟게 한다.
                    AppButton {
                        width: 100; height: 26
                        accent: true
                        text: "불러와 수정"
                        enabled: !Backend.jobActive
                        ToolTip.visible: hovered
                        ToolTip.text: "이 도면을 편집 상태로 불러옵니다.\n"
                                    + "그대로 다시 그리려면 불러온 뒤 [도면 전송] → [그림그리기 시작]"
                        onClicked: { if (Backend.loadJob(card.modelData.id)) dlg.close() }
                    }
                    Row {
                        spacing: 4
                        AppButton {
                            width: 44; height: 22
                            text: "이름"
                            onClicked: {
                                dlg.renamingId = (dlg.renamingId === card.modelData.id)
                                                 ? "" : card.modelData.id
                                dlg.confirmDeleteId = ""
                            }
                        }
                        AppButton {
                            width: 44; height: 22
                            danger: dlg.confirmDeleteId === card.modelData.id
                            text: dlg.confirmDeleteId === card.modelData.id ? "확인" : "삭제"
                            ToolTip.visible: hovered
                            ToolTip.text: "한 번 더 누르면 삭제됩니다"
                            onClicked: {
                                if (dlg.confirmDeleteId === card.modelData.id) {
                                    Backend.deleteJob(card.modelData.id)
                                    dlg.confirmDeleteId = ""
                                } else {
                                    dlg.confirmDeleteId = card.modelData.id
                                    dlg.renamingId = ""
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

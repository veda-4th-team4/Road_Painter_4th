import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

// 프리셋 경로 선택 다이얼로그. 미리보기와 함께 도형을 고른다.
Dialog {
    id: root

    property var library: null
    signal applied(string presetId)

    readonly property var items: [
        { pid: "line",      name: "수평 직선",          desc: "정점 2개 — 직선 구간 마킹" },
        { pid: "rectangle", name: "직사각형",           desc: "정점 4개 — 구역 외곽선 (폐곡선 권장)" },
        { pid: "triangle",  name: "삼각형",             desc: "정점 3개 (폐곡선 권장)" },
        { pid: "hexagon",   name: "원형 루프 (육각형)",  desc: "정점 6개 — 원형 근사 (폐곡선 권장)" },
        { pid: "parking",   name: "주차 구획 (4칸)",     desc: "스톨 구획선 한붓그리기 경로" }
    ]
    property int currentIndex: 0

    parent: Overlay.overlay
    anchors.centerIn: parent
    width: 430
    modal: true
    padding: 20
    focus: true

    background: Rectangle {
        color: Theme.bg1
        radius: 14
        border.color: Theme.stroke
        border.width: 1
    }

    contentItem: Column {
        spacing: Theme.sp3

        Text {
            text: "프리셋 경로 선택"
            color: Theme.textHi
            font.pixelSize: Theme.fontLg
            font.bold: true
        }
        Text {
            text: "선택한 도형이 작업 구역 중앙에 생성됩니다. 생성 후 드래그로 이동하거나 핸들로 크기를 조절하세요."
            color: Theme.textMid
            font.pixelSize: Theme.fontSm
            width: parent.width
            wrapMode: Text.WordWrap
        }

        Column {
            width: parent.width
            spacing: 6

            Repeater {
                model: root.items

                delegate: Rectangle {
                    id: row
                    required property int index
                    required property var modelData
                    readonly property bool selected: root.currentIndex === index
                    width: parent.width
                    height: 62
                    radius: 9
                    color: selected ? Qt.alpha(Theme.accent, 0.13)
                         : rowMa.containsMouse ? Theme.bg3 : Theme.bg2
                    border.color: selected ? Qt.alpha(Theme.accent, 0.6) : Theme.strokeSoft
                    border.width: 1

                    Row {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 14

                        Rectangle {
                            width: 74
                            height: parent.height
                            radius: 5
                            color: "#0a0c10"
                            border.color: Theme.strokeSoft

                            Canvas {
                                anchors.fill: parent
                                onPaint: {
                                    const ctx = getContext("2d")
                                    ctx.clearRect(0, 0, width, height)
                                    if (!root.library)
                                        return
                                    const pts = root.library.generate(
                                                  row.modelData.pid,
                                                  width / 2, height / 2,
                                                  Math.min(width, height) * 0.36)
                                    if (pts.length < 2)
                                        return
                                    ctx.strokeStyle = row.selected ? Theme.accent : Theme.pathLine
                                    ctx.lineWidth = 1.6
                                    ctx.lineJoin = "round"
                                    ctx.beginPath()
                                    ctx.moveTo(pts[0].x, pts[0].y)
                                    for (let i = 1; i < pts.length; ++i)
                                        ctx.lineTo(pts[i].x, pts[i].y)
                                    if (row.modelData.pid !== "line" && row.modelData.pid !== "parking")
                                        ctx.closePath()
                                    ctx.stroke()
                                }
                                Component.onCompleted: requestPaint()
                            }
                        }

                        Column {
                            anchors.verticalCenter: parent.verticalCenter
                            spacing: 3

                            Text {
                                text: row.modelData.name
                                color: row.selected ? Theme.textHi : Theme.textHi
                                font.pixelSize: Theme.fontMd
                                font.bold: row.selected
                            }
                            Text {
                                text: row.modelData.desc
                                color: Theme.textMid
                                font.pixelSize: Theme.fontXs
                            }
                        }
                    }

                    MouseArea {
                        id: rowMa
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        onClicked: root.currentIndex = row.index
                        onDoubleClicked: {
                            root.currentIndex = row.index
                            root.applied(root.items[root.currentIndex].pid)
                            root.close()
                        }
                    }
                }
            }
        }

        Row {
            anchors.right: parent.right
            spacing: Theme.sp2

            AppButton {
                text: "취소"
                onClicked: root.close()
            }
            AppButton {
                text: "선택 적용"
                variant: "primary"
                onClicked: {
                    root.applied(root.items[root.currentIndex].pid)
                    root.close()
                }
            }
        }
    }

    Overlay.modal: Rectangle {
        color: "#000000aa"
    }
}

import QtQuick
import RoadPainter

// 4채널 관제 컨셉 목업: CH1은 라이브 미니뷰, 나머지는 NO SIGNAL.
Item {
    id: root

    property Item liveItem: null

    implicitHeight: grid.implicitHeight

    Grid {
        id: grid
        width: parent.width
        columns: 2
        columnSpacing: Theme.sp2
        rowSpacing: Theme.sp2

        Repeater {
            model: 4

            delegate: Rectangle {
                id: cell
                required property int index
                readonly property bool isLive: index === 0
                width: (grid.width - grid.columnSpacing) / 2
                height: width * 9 / 16
                color: "#06070a"
                radius: Theme.radiusSmall
                border.color: isLive ? Qt.alpha(Theme.ok, 0.45) : Theme.strokeSoft
                border.width: 1
                clip: true

                ShaderEffectSource {
                    anchors.fill: parent
                    anchors.margins: 1
                    visible: cell.isLive && root.liveItem !== null
                    sourceItem: root.liveItem
                    live: true
                }

                Canvas {
                    anchors.fill: parent
                    visible: !cell.isLive
                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.fillStyle = "#0a0c10"
                        ctx.fillRect(0, 0, width, height)
                        for (let i = 0; i < 260; ++i) {
                            ctx.fillStyle = "rgba(255,255,255," + (Math.random() * 0.09).toFixed(3) + ")"
                            ctx.fillRect(Math.random() * width, Math.random() * height, 1.5, 1.5)
                        }
                    }
                }

                Text {
                    visible: !cell.isLive
                    anchors.centerIn: parent
                    text: "NO SIGNAL"
                    color: Theme.textLow
                    font.pixelSize: Theme.fontXs
                    font.family: Theme.mono
                    font.letterSpacing: 1.5
                }

                Row {
                    x: 6
                    anchors.bottom: parent.bottom
                    anchors.bottomMargin: 5
                    spacing: 5

                    Rectangle {
                        width: 6; height: 6; radius: 3
                        anchors.verticalCenter: parent.verticalCenter
                        color: cell.isLive ? Theme.ok : Theme.textLow
                    }
                    Text {
                        text: "CH" + (cell.index + 1) + (cell.isLive ? " · TOP-VIEW" : "")
                        color: cell.isLive ? Theme.textHi : Theme.textLow
                        font.pixelSize: Theme.fontXs
                        font.family: Theme.mono
                        style: Text.Outline
                        styleColor: "#000000"
                    }
                }
            }
        }
    }
}

import QtQuick
import RoadPainter

// 로봇 목업: 원형 섀시 + 상부 ArUco 플레이트 + 진행 방향 표시.
// rotation은 +x 방향(오른쪽)을 heading 0도로 가정한다.
Item {
    id: root

    property bool penDown: false
    property color penColor: "#f2f4f6"

    width: 52
    height: 52

    // 그림자
    Rectangle {
        anchors.centerIn: parent
        anchors.verticalCenterOffset: 3
        width: 44; height: 44; radius: 22
        color: "#000000"
        opacity: 0.35
    }

    // 섀시
    Rectangle {
        anchors.centerIn: parent
        width: 44; height: 44; radius: 22
        color: "#22262d"
        border.color: "#454c58"
        border.width: 2
    }

    // 펜 상태 링
    Rectangle {
        anchors.centerIn: parent
        width: 50; height: 50; radius: 25
        color: "transparent"
        border.color: root.penDown ? root.penColor : Theme.ok
        border.width: root.penDown ? 2.5 : 1.5
        opacity: root.penDown ? 0.95 : 0.45
    }

    // 상부 ArUco 플레이트
    Rectangle {
        anchors.centerIn: parent
        width: 28; height: 28; radius: 3
        color: "#e8eaef"

        Rectangle {
            anchors.centerIn: parent
            width: 22; height: 22
            color: "#0c0e11"

            Grid {
                anchors.centerIn: parent
                rows: 4
                columns: 4

                Repeater {
                    model: [1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0]
                    Rectangle {
                        required property int modelData
                        width: 4.5; height: 4.5
                        color: modelData ? "#e8eaef" : "#0c0e11"
                    }
                }
            }
        }
    }

    // 진행 방향 웨지 (+x)
    Canvas {
        x: parent.width - 10
        y: parent.height / 2 - 6
        width: 10
        height: 12
        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            ctx.fillStyle = Theme.accent
            ctx.beginPath()
            ctx.moveTo(0, 0)
            ctx.lineTo(width, height / 2)
            ctx.lineTo(0, height)
            ctx.closePath()
            ctx.fill()
        }
    }
}

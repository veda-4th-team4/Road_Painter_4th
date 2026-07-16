import QtQuick
import RoadPainter

// 하단 중앙 토스트 알림. show(message, kind) 로 호출. kind: "ok"|"warn"|"error"|"info"
Rectangle {
    id: root

    property string kind: "info"

    function show(message, k) {
        msgText.text = message
        kind = k || "info"
        opacity = 1
        hideTimer.restart()
    }

    readonly property color inkColor: kind === "ok" ? Theme.ok
                                    : kind === "warn" ? Theme.warn
                                    : kind === "error" ? Theme.danger
                                    : Theme.textHi

    width: row.width + 34
    height: 40
    radius: 20
    color: "#0e1116f0"
    border.color: Qt.alpha(inkColor, 0.55)
    border.width: 1
    opacity: 0
    visible: opacity > 0

    Behavior on opacity { NumberAnimation { duration: 180 } }

    Timer {
        id: hideTimer
        interval: 2800
        onTriggered: root.opacity = 0
    }

    Row {
        id: row
        anchors.centerIn: parent
        spacing: 8

        Rectangle {
            width: 8; height: 8; radius: 4
            anchors.verticalCenter: parent.verticalCenter
            color: root.inkColor
        }
        Text {
            id: msgText
            color: Theme.textHi
            font.pixelSize: Theme.fontSm
        }
    }
}

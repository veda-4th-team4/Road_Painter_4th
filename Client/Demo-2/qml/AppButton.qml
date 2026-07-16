import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Button {
    id: control

    // "default" | "primary" | "danger"
    property string variant: "default"

    padding: 7
    leftPadding: 14
    rightPadding: 14
    font.pixelSize: Theme.fontSm
    hoverEnabled: true

    contentItem: Text {
        text: control.text
        font: control.font
        color: control.variant === "primary" ? "#1c1206"
             : control.variant === "danger" ? "#ffd9d9"
             : Theme.textHi
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        opacity: control.enabled ? 1 : 0.5
    }

    background: Rectangle {
        radius: 7
        color: control.variant === "primary"
                 ? (control.down ? Qt.darker(Theme.accent, 1.25)
                    : control.hovered ? Qt.lighter(Theme.accent, 1.1) : Theme.accent)
             : control.variant === "danger"
                 ? (control.down ? "#5a2424" : control.hovered ? "#4a2020" : "#391c1c")
                 : (control.down || control.hovered ? Theme.bg3 : Theme.bg2)
        border.color: control.variant === "primary" ? "transparent"
                    : control.variant === "danger" ? "#5d2e2e"
                    : Theme.stroke
        border.width: 1
        opacity: control.enabled ? 1 : 0.45
    }
}

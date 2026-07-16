import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Switch {
    id: control

    font.pixelSize: Theme.fontSm
    hoverEnabled: true

    indicator: Rectangle {
        implicitWidth: 36
        implicitHeight: 20
        x: control.leftPadding
        y: parent.height / 2 - height / 2
        radius: 10
        color: control.checked ? Qt.alpha(Theme.accent, 0.85) : Theme.bg3
        border.color: control.checked ? Theme.accent : Theme.stroke

        Rectangle {
            x: control.checked ? parent.width - width - 2 : 2
            anchors.verticalCenter: parent.verticalCenter
            width: 16
            height: 16
            radius: 8
            color: "#f2f4f7"
            Behavior on x { NumberAnimation { duration: 110 } }
        }
    }

    contentItem: Text {
        text: control.text
        font: control.font
        color: control.hovered ? Theme.textHi : Theme.textMid
        verticalAlignment: Text.AlignVCenter
        leftPadding: control.indicator.width + 8
    }
}

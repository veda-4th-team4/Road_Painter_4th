import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Button {
    id: control
    property bool accent: false
    property bool danger: false
    property bool outline: false
    property color baseColor: accent ? Theme.accent
                          : (danger ? Theme.danger : Theme.elevated)

    implicitHeight: 36
    implicitWidth: Math.max(48, contentText.implicitWidth + 22)
    padding: 8
    hoverEnabled: true

    contentItem: Text {
        id: contentText
        text: control.text
        color: !control.enabled ? Theme.muted
             : (control.outline && control.danger) ? Theme.danger
             : (control.outline && control.accent) ? Theme.accent
             : (control.accent || control.danger) ? "#FFFFFF"
             : Theme.text
        font.pixelSize: 13
        font.bold: control.accent || control.danger
        font.family: Theme.fontFamily
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        radius: Theme.radius
        color: {
            if (!control.enabled) return Theme.elevated
            if (control.outline) return "transparent"
            if (control.down) return Qt.darker(control.baseColor, 1.15)
            if (control.hovered) return Qt.lighter(control.baseColor, 1.08)
            return control.baseColor
        }
        border.width: control.outline ? 1 : ((control.accent || control.danger) ? 0 : 1)
        border.color: control.outline
            ? (control.danger ? Theme.danger : (control.accent ? Theme.accent : Theme.stroke))
            : Theme.stroke
        opacity: control.enabled ? 1.0 : 0.55
    }
}

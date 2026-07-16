import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Column {
    id: root

    property string label: ""
    property real from: -100
    property real to: 100
    property real value: 0
    property string suffix: ""
    signal moved(real newValue)

    spacing: 4

    Row {
        width: parent.width
        Text {
            text: root.label
            color: Theme.textMid
            font.pixelSize: Theme.fontSm
            width: parent.width - valueText.width
            elide: Text.ElideRight
        }
        Text {
            id: valueText
            text: Math.round(slider.value) + root.suffix
            color: slider.pressed || slider.hovered ? Theme.accent : Theme.textHi
            font.pixelSize: Theme.fontSm
            font.family: Theme.mono
            horizontalAlignment: Text.AlignRight
        }
    }

    Slider {
        id: slider
        width: parent.width
        height: 20
        from: root.from
        to: root.to
        hoverEnabled: true
        onMoved: root.moved(value)

        // 외부에서 value가 바뀌어도(리셋 등) 슬라이더에 반영되도록 Binding 사용
        Binding {
            target: slider
            property: "value"
            value: root.value
        }

        background: Rectangle {
            x: slider.leftPadding
            y: slider.topPadding + slider.availableHeight / 2 - height / 2
            width: slider.availableWidth
            height: 4
            radius: 2
            color: Theme.bg3

            Rectangle {
                width: slider.visualPosition * parent.width
                height: parent.height
                radius: 2
                color: Qt.alpha(Theme.accent, 0.85)
            }
        }

        handle: Rectangle {
            x: slider.leftPadding + slider.visualPosition * (slider.availableWidth - width)
            y: slider.topPadding + slider.availableHeight / 2 - height / 2
            width: 14
            height: 14
            radius: 7
            color: slider.pressed ? Theme.accent : "#dfe3ea"
            border.color: slider.hovered || slider.pressed ? Theme.accent : Theme.stroke
            border.width: 1.5
        }
    }
}

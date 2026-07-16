import QtQuick
import RoadPainter

Rectangle {
    id: root

    property string title: ""
    default property alias content: inner.data

    width: parent ? parent.width : implicitWidth
    implicitHeight: col.implicitHeight + 2 * 14
    color: Theme.bg1
    radius: Theme.radius
    border.color: Theme.strokeSoft
    border.width: 1

    Column {
        id: col
        anchors.fill: parent
        anchors.margins: 14
        spacing: Theme.sp3

        Text {
            text: root.title
            color: Theme.textLow
            font.pixelSize: Theme.fontXs
            font.bold: true
            font.letterSpacing: 1.4
        }

        Column {
            id: inner
            width: parent.width
            spacing: Theme.sp3
        }
    }
}

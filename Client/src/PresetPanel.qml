import QtQuick
import RoadPainter

Rectangle {
    id: panel
    width: 300
    height: 148
    radius: Theme.radius
    color: Theme.surface
    border.color: Theme.stroke
    border.width: 1
    visible: false
    z: 100

    property Item rootItem: null

    function showAt(px, py) { x = px; y = py; visible = true }

    Rectangle {
        id: handle
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 34
        color: Theme.elevated

        Text {
            anchors.left: parent.left
            anchors.leftMargin: 10
            anchors.verticalCenter: parent.verticalCenter
            text: "도형 프리셋"
            color: Theme.text
            font.pixelSize: 12
            font.bold: true
            font.family: Theme.fontFamily
        }

        MouseArea {
            anchors.fill: parent
            anchors.rightMargin: 28
            cursorShape: Qt.SizeAllCursor
            drag.target: panel
            drag.axis: Drag.XAndYAxis
        }

        Text {
            anchors.right: parent.right
            anchors.rightMargin: 10
            anchors.verticalCenter: parent.verticalCenter
            text: "×"
            color: Theme.sub
            font.pixelSize: 16
            MouseArea {
                anchors.fill: parent
                anchors.margins: -6
                cursorShape: Qt.PointingHandCursor
                onClicked: panel.visible = false
            }
        }
    }

    Row {
        anchors.top: handle.bottom
        anchors.topMargin: 12
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 10
        PresetTile { shape: "RECT"; label: "직사각형"; rootItem: panel.rootItem }
        PresetTile { shape: "TRIANGLE"; label: "삼각형"; rootItem: panel.rootItem }
        PresetTile { shape: "CIRCLE"; label: "루프"; rootItem: panel.rootItem }
        PresetTile { shape: "LINE"; label: "직선"; rootItem: panel.rootItem }
    }
}

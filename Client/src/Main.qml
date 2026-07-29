import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

ApplicationWindow {
    id: win
    visible: true
    width: 1440
    height: 900
    minimumWidth: 1200
    minimumHeight: 720
    title: "Road Painter"
    color: Theme.bg

    StackView {
        id: stack
        anchors.fill: parent
        initialItem: loginComp
    }

    Component { id: loginComp; LoginPage {} }
    Component { id: mainComp;  MainPage {} }

    Connections {
        target: Backend
        function onLoginSucceeded() { stack.replace(mainComp) }
        function onLoggedOut() { stack.replace(loginComp) }
    }
}

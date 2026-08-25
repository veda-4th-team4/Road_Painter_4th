import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

ApplicationWindow {
    id: win
    property bool mainPageShown: false
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
        function onLoginSucceeded() {
            if (win.mainPageShown) return
            win.mainPageShown = true
            stack.replace(mainComp)
        }
        function onLoggedOut() {
            if (!win.mainPageShown) return
            win.mainPageShown = false
            stack.replace(loginComp)
        }
    }
}

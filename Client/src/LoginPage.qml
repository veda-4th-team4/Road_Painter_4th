import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Item {
    id: root

    Rectangle {
        anchors.fill: parent
        color: Theme.bg
    }

    Rectangle {
        id: card
        width: 400
        anchors.centerIn: parent
        color: Theme.surface
        border.color: Theme.stroke
        border.width: 1
        radius: Theme.radius
        implicitHeight: cardCol.implicitHeight + 48

        Column {
            id: cardCol
            width: parent.width - 56
            anchors.centerIn: parent
            spacing: 12

            Image {
                source: "qrc:/assets/hanwha_vision_logo.png"
                width: Math.min(200, parent.width)
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Text {
                text: "Road Painter"
                color: Theme.text
                font.pixelSize: 22
                font.bold: true
                font.family: Theme.fontFamily
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text {
                text: "관제 클라이언트"
                color: Theme.sub
                font.pixelSize: 13
                font.family: Theme.fontFamily
                anchors.horizontalCenter: parent.horizontalCenter
            }
            Text {
                text: "192.168.0.8:9000  ·  test / test"
                color: Theme.muted
                font.pixelSize: 11
                font.family: Theme.fontFamily
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Item { width: 1; height: 4 }

            Text { text: "아이디"; color: Theme.sub; font.pixelSize: 12; font.family: Theme.fontFamily }
            TextField {
                id: idField
                width: parent.width
                placeholderText: "아이디"
                placeholderTextColor: Theme.muted
                color: Theme.text
                selectByMouse: true
                leftPadding: 10
                font.family: Theme.fontFamily
                onAccepted: pwField.forceActiveFocus()
                background: Rectangle {
                    implicitHeight: 40
                    radius: Theme.radius
                    color: Theme.panel
                    border.width: 1
                    border.color: idField.activeFocus ? Theme.accent : Theme.stroke
                }
            }

            Text { text: "비밀번호"; color: Theme.sub; font.pixelSize: 12; font.family: Theme.fontFamily }
            TextField {
                id: pwField
                width: parent.width
                placeholderText: "비밀번호"
                placeholderTextColor: Theme.muted
                color: Theme.text
                echoMode: TextInput.Password
                selectByMouse: true
                leftPadding: 10
                font.family: Theme.fontFamily
                onAccepted: root.doLogin()
                background: Rectangle {
                    implicitHeight: 40
                    radius: Theme.radius
                    color: Theme.panel
                    border.width: 1
                    border.color: pwField.activeFocus ? Theme.accent : Theme.stroke
                }
            }

            Text {
                id: errorText
                width: parent.width
                text: ""
                color: Theme.danger
                font.pixelSize: 12
                font.family: Theme.fontFamily
                wrapMode: Text.WordWrap
                visible: text.length > 0
            }

            AppButton {
                width: parent.width
                height: 42
                accent: true
                enabled: !Backend.busy
                text: Backend.busy ? "로그인 중…" : "로그인"
                onClicked: root.doLogin()
            }
            AppButton {
                width: parent.width
                text: "회원가입"
                onClicked: signupDialog.open()
            }
        }
    }

    function doLogin() {
        errorText.text = ""
        Backend.login(idField.text.trim(), pwField.text)
    }

    Connections {
        target: Backend
        function onLoginFailed(reason) { errorText.text = reason }
    }

    SignupDialog { id: signupDialog }
    Component.onCompleted: idField.forceActiveFocus()
}

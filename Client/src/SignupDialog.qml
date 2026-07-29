import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

Popup {
    id: root
    modal: true
    dim: false
    parent: Overlay.overlay
    x: 0
    y: 0
    width: parent ? parent.width : 800
    height: parent ? parent.height : 600
    padding: 0
    closePolicy: Popup.CloseOnEscape

    onOpened: {
        idField.clear(); pwField.clear(); pw2Field.clear(); camField.clear()
        msg.text = ""
        idField.forceActiveFocus()
    }

    background: Rectangle { color: Theme.bg }

    contentItem: Item {
        Rectangle {
            id: card
            anchors.centerIn: parent
            width: 400
            radius: Theme.radius
            color: Theme.surface
            border.color: Theme.stroke
            border.width: 1
            implicitHeight: cardCol.implicitHeight + 40

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
                    text: "회원가입"
                    color: Theme.text
                    font.pixelSize: 18
                    font.bold: true
                    font.family: Theme.fontFamily
                    anchors.horizontalCenter: parent.horizontalCenter
                }

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
                        implicitHeight: 40; radius: Theme.radius; color: Theme.panel
                        border.width: 1; border.color: idField.activeFocus ? Theme.accent : Theme.stroke
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
                    onAccepted: pw2Field.forceActiveFocus()
                    background: Rectangle {
                        implicitHeight: 40; radius: Theme.radius; color: Theme.panel
                        border.width: 1; border.color: pwField.activeFocus ? Theme.accent : Theme.stroke
                    }
                }

                Text { text: "비밀번호 확인"; color: Theme.sub; font.pixelSize: 12; font.family: Theme.fontFamily }
                TextField {
                    id: pw2Field
                    width: parent.width
                    placeholderText: "비밀번호 다시 입력"
                    placeholderTextColor: Theme.muted
                    color: Theme.text
                    echoMode: TextInput.Password
                    selectByMouse: true
                    leftPadding: 10
                    font.family: Theme.fontFamily
                    onAccepted: root.doRegister()
                    background: Rectangle {
                        implicitHeight: 40; radius: Theme.radius; color: Theme.panel
                        border.width: 1; border.color: pw2Field.activeFocus ? Theme.accent : Theme.stroke
                    }
                }

                Text { text: "카메라 IP (선택)"; color: Theme.sub; font.pixelSize: 12; font.family: Theme.fontFamily }
                TextField {
                    id: camField
                    width: parent.width
                    placeholderText: "192.168.0.9 — 서버에 저장되어 로그인 시 회신됩니다"
                    placeholderTextColor: Theme.muted
                    color: Theme.text
                    selectByMouse: true
                    leftPadding: 10
                    font.family: Theme.fontFamily
                    onAccepted: root.doRegister()
                    background: Rectangle {
                        implicitHeight: 40; radius: Theme.radius; color: Theme.panel
                        border.width: 1; border.color: camField.activeFocus ? Theme.accent : Theme.stroke
                    }
                }

                Text {
                    id: msg
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
                    accent: true
                    enabled: !Backend.busy
                    text: Backend.busy ? "가입 중…" : "가입"
                    onClicked: root.doRegister()
                }
                AppButton {
                    width: parent.width
                    text: "취소"
                    onClicked: root.close()
                }
            }
        }
    }

    function doRegister() {
        msg.text = ""
        msg.color = Theme.danger
        if (pwField.text !== pw2Field.text) { msg.text = "비밀번호가 일치하지 않습니다."; return }
        Backend.registerUser(idField.text.trim(), pwField.text, camField.text.trim())
    }

    Connections {
        target: Backend
        function onRegisterSucceeded() { msg.color = Theme.success; msg.text = "회원가입 완료. 로그인해 주세요."; closeTimer.start() }
        function onRegisterFailed(reason) { msg.color = Theme.danger; msg.text = reason }
    }
    Timer { id: closeTimer; interval: 1200; onTriggered: root.close() }
}

import QtQuick
import QtQuick.Controls.Basic
import RoadPainter

// 경로 작도 완료 다이얼로그: 임시 저장 / 파일로 내보내기 / 계속 편집.
Dialog {
    id: root

    property int pointCount: 0
    signal saveTemp()
    signal exportRequested()

    parent: Overlay.overlay
    anchors.centerIn: parent
    width: 400
    modal: true
    padding: 20
    focus: true

    background: Rectangle {
        color: Theme.bg1
        radius: 14
        border.color: Theme.stroke
        border.width: 1
    }

    contentItem: Column {
        spacing: Theme.sp4

        Text {
            text: "경로 작도 완료"
            color: Theme.textHi
            font.pixelSize: Theme.fontLg
            font.bold: true
        }

        Text {
            text: "정점 " + root.pointCount + "개의 경로가 작도되었습니다.\n로봇 제어용 JSON으로 저장하시겠습니까?"
            color: Theme.textMid
            font.pixelSize: Theme.fontSm
            width: parent.width
            wrapMode: Text.WordWrap
            lineHeight: 1.3
        }

        Row {
            width: parent.width
            spacing: Theme.sp2

            AppButton {
                width: (parent.width - 2 * Theme.sp2) / 3
                text: "임시 저장"
                onClicked: { root.saveTemp(); root.close() }
            }
            AppButton {
                width: (parent.width - 2 * Theme.sp2) / 3
                text: "파일로 내보내기"
                variant: "primary"
                onClicked: { root.exportRequested(); root.close() }
            }
            AppButton {
                width: (parent.width - 2 * Theme.sp2) / 3
                text: "계속 편집"
                onClicked: root.close()
            }
        }
    }

    Overlay.modal: Rectangle {
        color: "#000000aa"
    }
}

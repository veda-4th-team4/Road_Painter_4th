import QtQuick
import RoadPainter

// 목업 CCTV 영상 + 도장 궤적 + 로봇 + 필터(ShaderEffect) + 작도 오버레이.
// stage는 스트림 이미지 좌표계(1280x720) 그대로이며 뷰포트에 맞춰 스케일된다.
Item {
    id: root

    property string mode: "select"
    property var pathModel
    property var robot
    property var filters
    property color trailColor: "#f2f4f6"

    readonly property point imageCursor: overlay.cursorPos
    readonly property Item videoContentItem: videoContent

    signal drawFinished()

    function clearTrail() {
        trailCanvas.segments = []
        trailCanvas.requestPaint()
    }

    Rectangle {
        anchors.fill: parent
        color: "#040507"
    }

    Item {
        id: stage
        width: root.pathModel ? root.pathModel.imageWidth : 1280
        height: root.pathModel ? root.pathModel.imageHeight : 720
        anchors.centerIn: parent
        scale: Math.min(root.width / width, root.height / height)

        Item {
            id: videoContent
            anchors.fill: parent

            CctvScene {
                anchors.fill: parent
            }

            // 로봇 펜이 바닥에 남긴 도장 궤적
            Canvas {
                id: trailCanvas
                anchors.fill: parent
                property var segments: []
                onPaint: {
                    const ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    ctx.lineCap = "round"
                    ctx.lineJoin = "round"
                    ctx.lineWidth = 5
                    for (let i = 0; i < segments.length; ++i) {
                        const s = segments[i]
                        ctx.strokeStyle = s.c
                        ctx.beginPath()
                        ctx.moveTo(s.x1, s.y1)
                        ctx.lineTo(s.x2, s.y2)
                        ctx.stroke()
                    }
                }
            }

            RobotMarker {
                x: (root.robot ? root.robot.x : 90) - width / 2
                y: (root.robot ? root.robot.y : 640) - height / 2
                rotation: root.robot ? root.robot.heading : 0
                penDown: root.robot ? root.robot.penDown : false
                penColor: root.trailColor

                Behavior on rotation {
                    RotationAnimation {
                        duration: 100
                        direction: RotationAnimation.Shortest
                    }
                }
            }
        }

        ShaderEffectSource {
            id: videoSource
            sourceItem: videoContent
            hideSource: true
            live: true
            smooth: true
        }

        ShaderEffect {
            anchors.fill: parent
            fragmentShader: "qrc:/shaders/videofilter.frag.qsb"
            property var source: videoSource
            property real brightness: root.filters ? root.filters.brightness / 100 : 0
            property real contrast: root.filters ? root.filters.contrast / 100 : 0
            property real saturation: root.filters ? root.filters.saturation / 100 : 0
            property real sharpen: root.filters ? root.filters.sharpen / 100 : 0
            property real grayscale: root.filters && root.filters.grayscale ? 1.0 : 0.0
            property point texelStep: Qt.point(1.0 / stage.width, 1.0 / stage.height)
        }

        PathOverlay {
            id: overlay
            anchors.fill: parent
            mode: root.mode
            model: root.pathModel
            onFinishRequested: root.drawFinished()
        }
    }

    Connections {
        target: root.robot
        function onTrailSegment(x1, y1, x2, y2) {
            trailCanvas.segments.push({ x1: x1, y1: y1, x2: x2, y2: y2,
                                        c: root.trailColor.toString() })
            trailCanvas.requestPaint()
        }
        function onTrailCleared() {
            root.clearTrail()
        }
    }

    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.color: Theme.stroke
        border.width: 1
    }
}

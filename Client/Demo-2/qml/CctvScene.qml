import QtQuick
import RoadPainter

// RTSP 스트림을 대체하는 목업 CCTV 탑뷰 씬 (지하주차장 바닥).
// 정적 바닥 + OSD(타임스탬프/REC) 조합으로 실제 관제 영상처럼 보이게 한다.
Item {
    id: root

    Canvas {
        id: floor
        anchors.fill: parent

        // 시드 고정 의사난수 → 항상 같은 씬이 그려진다.
        function makeRng(seed) {
            let s = seed >>> 0
            return function () {
                s = (s + 0x6D2B79F5) >>> 0
                let t = Math.imul(s ^ (s >>> 15), 1 | s)
                t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t
                return ((t ^ (t >>> 14)) >>> 0) / 4294967296
            }
        }

        function drawAruco(ctx, x, y, size, rng) {
            ctx.fillStyle = "#e8eaec"
            ctx.fillRect(x, y, size, size)
            ctx.fillStyle = "#101214"
            const m = size / 8
            ctx.fillRect(x + m, y + m, size - 2 * m, size - 2 * m)
            const cell = (size - 4 * m) / 4
            for (let r = 0; r < 4; ++r)
                for (let c = 0; c < 4; ++c)
                    if (rng() > 0.5) {
                        ctx.fillStyle = "#e8eaec"
                        ctx.fillRect(x + 2 * m + c * cell, y + 2 * m + r * cell, cell, cell)
                    }
        }

        onPaint: {
            const ctx = getContext("2d")
            const w = width
            const h = height
            const rng = makeRng(20260716)

            // 콘크리트 바닥
            const g = ctx.createLinearGradient(0, 0, w, h)
            g.addColorStop(0, "#565b62")
            g.addColorStop(0.55, "#4a4e55")
            g.addColorStop(1, "#3f434a")
            ctx.fillStyle = g
            ctx.fillRect(0, 0, w, h)

            // 타일 줄눈
            ctx.strokeStyle = "rgba(0,0,0,0.14)"
            ctx.lineWidth = 1.5
            for (let x = 0; x <= w; x += 128) {
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke()
            }
            for (let y = 0; y <= h; y += 120) {
                ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke()
            }

            // 얼룩
            for (let i = 0; i < 14; ++i) {
                const sx = rng() * w, sy = rng() * h
                const rx = 30 + rng() * 90, ry = 20 + rng() * 50
                ctx.fillStyle = "rgba(0,0,0," + (0.04 + rng() * 0.07).toFixed(3) + ")"
                ctx.beginPath()
                ctx.ellipse(sx - rx / 2, sy - ry / 2, rx, ry)
                ctx.fill()
            }

            // 균열
            ctx.strokeStyle = "rgba(0,0,0,0.18)"
            ctx.lineWidth = 1
            for (let i = 0; i < 5; ++i) {
                let cx0 = rng() * w, cy0 = rng() * h
                ctx.beginPath(); ctx.moveTo(cx0, cy0)
                for (let s = 0; s < 6; ++s) {
                    cx0 += (rng() - 0.5) * 90
                    cy0 += (rng() - 0.5) * 60
                    ctx.lineTo(cx0, cy0)
                }
                ctx.stroke()
            }

            // 상단 주차 구획 (기존에 칠해진 라인들)
            ctx.strokeStyle = "rgba(228,231,235,0.8)"
            ctx.lineWidth = 4
            const stallTop = 6, stallBottom = 126
            for (let i = 0; i <= 8; ++i) {
                const x = 44 + i * 118
                ctx.beginPath(); ctx.moveTo(x, stallTop); ctx.lineTo(x, stallBottom); ctx.stroke()
            }
            ctx.beginPath(); ctx.moveTo(44, stallBottom); ctx.lineTo(44 + 8 * 118, stallBottom); ctx.stroke()

            // 주차된 차량 목업
            function car(x, y, color) {
                ctx.fillStyle = "rgba(0,0,0,0.25)"
                ctx.fillRect(x + 4, y + 6, 96, 102)
                ctx.fillStyle = color
                ctx.beginPath()
                ctx.roundedRect(x, y, 96, 102, 14, 14)
                ctx.fill()
                ctx.fillStyle = "rgba(210,220,230,0.35)"
                ctx.beginPath()
                ctx.roundedRect(x + 12, y + 14, 72, 22, 6, 6)
                ctx.fill()
            }
            car(55, 14, "#2e4a6a")
            car(409, 14, "#5a3038")
            car(881, 14, "#3b3f46")

            // 작업 구역 (호모그래피 기준 영역) — 주황 점선 + ArUco 기준 마커
            const wa = { x: 280, y: 180, w: 900, h: 480 }
            ctx.setLineDash([12, 9])
            ctx.strokeStyle = "rgba(255,122,26,0.75)"
            ctx.lineWidth = 2
            ctx.strokeRect(wa.x, wa.y, wa.w, wa.h)
            ctx.setLineDash([])

            drawAruco(ctx, wa.x + 8, wa.y + 8, 30, rng)
            drawAruco(ctx, wa.x + wa.w - 38, wa.y + 8, 30, rng)
            drawAruco(ctx, wa.x + 8, wa.y + wa.h - 38, 30, rng)
            drawAruco(ctx, wa.x + wa.w - 38, wa.y + wa.h - 38, 30, rng)

            ctx.fillStyle = "rgba(255,122,26,0.55)"
            ctx.font = "11px " + Theme.mono
            ctx.fillText("WORK AREA 8.0m x 4.5m", wa.x + 48, wa.y + 24)

            // 바닥 대형 문자
            ctx.save()
            ctx.fillStyle = "rgba(255,255,255,0.06)"
            ctx.font = "bold 120px sans-serif"
            ctx.fillText("B1", 1060, 690)
            ctx.restore()

            // 낡은 기존 차선 (희미한 노란 선)
            ctx.setLineDash([26, 18])
            ctx.strokeStyle = "rgba(230,190,60,0.22)"
            ctx.lineWidth = 6
            ctx.beginPath(); ctx.moveTo(60, 690); ctx.lineTo(w - 60, 690); ctx.stroke()
            ctx.setLineDash([])

            // 기둥 2개
            function pillar(x, y) {
                ctx.fillStyle = "#686d75"
                ctx.fillRect(x, y, 46, 46)
                ctx.strokeStyle = "rgba(0,0,0,0.35)"
                ctx.lineWidth = 2
                ctx.strokeRect(x, y, 46, 46)
                ctx.fillStyle = "rgba(240,190,40,0.75)"
                ctx.fillRect(x, y + 38, 46, 8)
                ctx.fillStyle = "rgba(20,20,20,0.8)"
                for (let s = 0; s < 3; ++s)
                    ctx.fillRect(x + 4 + s * 16, y + 38, 8, 8)
            }
            pillar(150, 380)
            pillar(1216, 380)

            // 센서 노이즈
            for (let i = 0; i < 1600; ++i) {
                ctx.fillStyle = "rgba(255,255,255," + (0.015 + rng() * 0.035).toFixed(3) + ")"
                ctx.fillRect(rng() * w, rng() * h, 1.2, 1.2)
            }

            // 스캔라인
            ctx.fillStyle = "rgba(255,255,255,0.014)"
            for (let y = 0; y < h; y += 4)
                ctx.fillRect(0, y, w, 1)

            // 비네트
            const v = ctx.createRadialGradient(w / 2, h / 2, h * 0.35, w / 2, h / 2, w * 0.72)
            v.addColorStop(0, "rgba(0,0,0,0)")
            v.addColorStop(1, "rgba(0,0,0,0.4)")
            ctx.fillStyle = v
            ctx.fillRect(0, 0, w, h)
        }
    }

    // ── CCTV OSD ─────────────────────────────────────
    Text {
        x: 18; y: 14
        text: "CAM 01 · B1 PARKING · TOP-VIEW"
        color: "#e6e9ed"
        font.pixelSize: 15
        font.family: Theme.mono
        style: Text.Outline
        styleColor: "#000000"
    }

    Row {
        anchors.right: parent.right
        anchors.rightMargin: 18
        y: 14
        spacing: 10

        Rectangle {
            width: 12; height: 12; radius: 6
            anchors.verticalCenter: parent.verticalCenter
            color: "#ff4545"
            SequentialAnimation on opacity {
                loops: Animation.Infinite
                NumberAnimation { to: 0.15; duration: 700 }
                NumberAnimation { to: 1.0; duration: 700 }
            }
        }
        Text {
            id: clockText
            text: "--"
            color: "#e6e9ed"
            font.pixelSize: 15
            font.family: Theme.mono
            style: Text.Outline
            styleColor: "#000000"

            Timer {
                interval: 500
                running: true
                repeat: true
                triggeredOnStart: true
                onTriggered: clockText.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
            }
        }
    }

    Text {
        x: 18
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 14
        text: "PNM-C16083RVQ (MOCK STREAM)"
        color: "#c9ced6"
        font.pixelSize: 12
        font.family: Theme.mono
        style: Text.Outline
        styleColor: "#000000"
        opacity: 0.85
    }

    Text {
        anchors.right: parent.right
        anchors.rightMargin: 18
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 14
        text: "H.264 · 1280x720 · 30FPS"
        color: "#c9ced6"
        font.pixelSize: 12
        font.family: Theme.mono
        style: Text.Outline
        styleColor: "#000000"
        opacity: 0.85
    }
}

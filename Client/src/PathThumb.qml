import QtQuick

// 작업 이력의 도면 미리보기.
//   mode "plan"   — 계획한 경로 전체 (before)
//   mode "result" — 실제로 칠해진 부분만 진하게, 남은 부분은 흐리게 (after)
// 좌표는 바닥 미터 좌표(BLUEPRINT 와 동일)라 y 가 위로 향한다 → 그릴 때 뒤집는다.
Canvas {
    id: thumb

    property var paths: []          // [[{x,y}, ...], ...]
    property var closedFlags: []
    property string mode: "plan"
    property real progress: 0.0

    onPathsChanged: requestPaint()
    onProgressChanged: requestPaint()
    onModeChanged: requestPaint()

    // 도형들을 BLUEPRINT 와 같은 규칙(닫힌 도형은 시작점 복귀)으로 한 줄로 잇는다.
    // 도형과 도형 사이를 잇는 구간은 실제 그림이 아니라 "이동"이므로 표시하지 않는다.
    function travelLine() {
        var line = [], join = []
        if (!paths) return { pts: line, join: join }
        for (var i = 0; i < paths.length; ++i) {
            var p = paths[i]
            if (!p || p.length < 2) continue
            for (var k = 0; k < p.length; ++k) {
                if (line.length > 0) join.push(k === 0)   // 도형 첫 점 = 이동 구간
                line.push(p[k])
            }
            if (closedFlags && closedFlags[i] && p.length > 2) {
                join.push(false)
                line.push(p[0])
            }
        }
        return { pts: line, join: join }
    }

    onPaint: {
        var ctx = getContext("2d")
        ctx.reset()
        ctx.clearRect(0, 0, width, height)

        var tl = travelLine()
        var line = tl.pts
        var join = tl.join
        if (line.length < 2) {
            ctx.fillStyle = "#5A626B"
            ctx.font = "10px sans-serif"
            ctx.fillText("경로 없음", 8, height / 2)
            return
        }

        // 바운딩 박스 → 여백을 둔 채로 맞춤
        var minX = line[0].x, maxX = line[0].x, minY = line[0].y, maxY = line[0].y
        for (var i = 1; i < line.length; ++i) {
            minX = Math.min(minX, line[i].x); maxX = Math.max(maxX, line[i].x)
            minY = Math.min(minY, line[i].y); maxY = Math.max(maxY, line[i].y)
        }
        var pad = 8
        var w = Math.max(maxX - minX, 1e-6), h = Math.max(maxY - minY, 1e-6)
        var s = Math.min((width - pad * 2) / w, (height - pad * 2) / h)
        var offX = (width - w * s) / 2 - minX * s
        var offY = (height - h * s) / 2 + maxY * s   // y 뒤집기

        function X(p) { return p.x * s + offX }
        function Y(p) { return -p.y * s + offY }

        // 전체 길이 → 진행률만큼만 진하게 칠한다
        var total = 0, segs = []
        for (i = 1; i < line.length; ++i) {
            var d = Math.hypot(line[i].x - line[i-1].x, line[i].y - line[i-1].y)
            segs.push(d); total += d
        }
        var painted = (mode === "result") ? total * progress : 0
        var acc = 0

        ctx.lineCap = "round"
        ctx.lineJoin = "round"

        // 바탕(계획) — result 모드에서는 아직 안 칠한 부분.
        // 도형 사이 이동 구간(join)은 건너뛴다.
        ctx.beginPath()
        for (i = 1; i < line.length; ++i) {
            if (join[i-1]) continue
            ctx.moveTo(X(line[i-1]), Y(line[i-1]))
            ctx.lineTo(X(line[i]), Y(line[i]))
        }
        ctx.strokeStyle = (mode === "result") ? "#3A424B" : "#E8EDF2"
        ctx.lineWidth = 2
        ctx.stroke()

        // 칠해진 구간
        if (mode === "result" && painted > 0) {
            ctx.beginPath()
            for (i = 1; i < line.length; ++i) {
                var d0 = segs[i-1]
                if (acc >= painted) break
                if (!join[i-1]) {
                    var t = Math.min(1, (painted - acc) / d0)
                    var px = line[i-1].x + (line[i].x - line[i-1].x) * t
                    var py = line[i-1].y + (line[i].y - line[i-1].y) * t
                    ctx.moveTo(X(line[i-1]), Y(line[i-1]))
                    ctx.lineTo(px * s + offX, -py * s + offY)
                }
                acc += d0
            }
            ctx.strokeStyle = (progress >= 0.999) ? "#FFFFFF" : "#F07020"
            ctx.lineWidth = 2.4
            ctx.stroke()
        }

        // 시작점
        ctx.beginPath()
        ctx.arc(X(line[0]), Y(line[0]), 2.6, 0, Math.PI * 2)
        ctx.fillStyle = "#00E5FF"
        ctx.fill()
    }
}

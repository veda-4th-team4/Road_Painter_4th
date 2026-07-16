pragma Singleton
import QtQuick

// 다크 톤 디자인 토큰 싱글턴.
QtObject {
    // ── 색상 ─────────────────────────────────────────
    readonly property color bg0: "#0b0d10"        // 앱 배경
    readonly property color bg1: "#12151a"        // 패널/카드
    readonly property color bg2: "#1a1e25"        // 컨트롤
    readonly property color bg3: "#232833"        // 호버/트랙
    readonly property color stroke: "#2b313d"     // 테두리
    readonly property color strokeSoft: "#20242d"

    readonly property color textHi: "#e8eaef"
    readonly property color textMid: "#9aa3b2"
    readonly property color textLow: "#5c6577"

    readonly property color accent: "#ff7a1a"     // Road-Painter 오렌지
    readonly property color ok: "#3ecf8e"
    readonly property color warn: "#ffb020"
    readonly property color danger: "#ff5d5d"
    readonly property color pathLine: "#37c5ff"   // 작도 오버레이

    // ── 간격/치수 ────────────────────────────────────
    readonly property int sp1: 4
    readonly property int sp2: 8
    readonly property int sp3: 12
    readonly property int sp4: 16
    readonly property int sp5: 24
    readonly property int radius: 10
    readonly property int radiusSmall: 6

    // ── 타이포 ───────────────────────────────────────
    readonly property int fontXs: 10
    readonly property int fontSm: 12
    readonly property int fontMd: 13
    readonly property int fontLg: 15
    readonly property int fontXl: 20
    readonly property string mono: "Consolas"
}

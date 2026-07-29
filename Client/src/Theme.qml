pragma Singleton
import QtQuick

// Hanwha Vision operator console — light work surface, brand orange #F07020
QtObject {
    readonly property color bg:        "#F5F6F8"
    readonly property color surface:   "#FFFFFF"
    readonly property color panel:     "#F0F2F5"
    readonly property color elevated:  "#E8EBEF"
    readonly property color stroke:    "#E2E5EA"

    readonly property color accent:    "#F07020"
    readonly property color accentDim: "#D45F18"
    readonly property color accentSoft: "#1AF07020"

    readonly property color text:  "#1A1D21"
    readonly property color sub:   "#5C6570"
    readonly property color muted: "#8B939C"

    readonly property color danger:  "#C62828"
    readonly property color success: "#2E7D32"
    readonly property color warn:    "#C77700"

    readonly property color okSoft:     "#E8F5E9"
    readonly property color warnSoft:   "#FFF3E0"
    readonly property color dangerSoft: "#FFEBEE"
    readonly property color mutedSoft:  "#EEF0F3"

    readonly property int radius: 6
    readonly property string fontFamily: "Pretendard"
}

#include "presetlibrary.h"

#include <QPointF>
#include <QVariantMap>
#include <QtMath>

namespace {
void append(QVariantList &list, qreal x, qreal y)
{
    list.append(QVariantMap{ { QStringLiteral("x"), x }, { QStringLiteral("y"), y } });
}
} // namespace

PresetLibrary::PresetLibrary(QObject *parent)
    : QObject(parent)
{
}

QVariantList PresetLibrary::generate(const QString &presetId, qreal cx, qreal cy, qreal size) const
{
    QVariantList pts;
    const qreal r = qMax<qreal>(10.0, size);

    if (presetId == QStringLiteral("line")) {
        append(pts, cx - r, cy);
        append(pts, cx + r, cy);
    } else if (presetId == QStringLiteral("rectangle")) {
        append(pts, cx - r, cy - r * 0.62);
        append(pts, cx + r, cy - r * 0.62);
        append(pts, cx + r, cy + r * 0.62);
        append(pts, cx - r, cy + r * 0.62);
    } else if (presetId == QStringLiteral("triangle")) {
        append(pts, cx, cy - r);
        append(pts, cx + r, cy + r * 0.75);
        append(pts, cx - r, cy + r * 0.75);
    } else if (presetId == QStringLiteral("hexagon")) {
        for (int i = 0; i < 6; ++i) {
            const qreal angle = i * (M_PI / 3.0);
            append(pts, cx + r * qCos(angle), cy + r * qSin(angle));
        }
    } else if (presetId == QStringLiteral("parking")) {
        // 주차 구획: 세로 구획선을 상/하 베이스라인으로 번갈아 잇는
        // 부스트로피던(boustrophedon) 한붓그리기 경로.
        const int stalls = 4;
        const qreal stallW = r * 0.55;
        const qreal depth = r * 1.1;
        const qreal x0 = cx - stalls * stallW / 2.0;
        const qreal top = cy - depth / 2.0;
        const qreal bottom = cy + depth / 2.0;

        append(pts, x0, top);
        append(pts, x0, bottom);
        for (int i = 1; i <= stalls; ++i) {
            const qreal xi = x0 + i * stallW;
            if (i % 2 == 1) {
                append(pts, xi, bottom);
                append(pts, xi, top);
            } else {
                append(pts, xi, top);
                append(pts, xi, bottom);
            }
        }
    }

    return pts;
}

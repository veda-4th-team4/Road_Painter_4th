#ifndef PRESETLIBRARY_H
#define PRESETLIBRARY_H

#include <QObject>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

// 미리 정의된 경로 프리셋(도형) 생성기.
// (cx, cy)를 중심으로 size를 기준 반경으로 하는 정점 목록을 돌려준다.
class PresetLibrary : public QObject
{
    Q_OBJECT
    QML_ELEMENT

public:
    explicit PresetLibrary(QObject *parent = nullptr);

    // presetId: "line" | "rectangle" | "triangle" | "hexagon" | "parking"
    Q_INVOKABLE QVariantList generate(const QString &presetId, qreal cx, qreal cy, qreal size) const;
};

#endif // PRESETLIBRARY_H

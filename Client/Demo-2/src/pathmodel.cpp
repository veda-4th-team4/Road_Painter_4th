#include "pathmodel.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
double roundTo(double v, int decimals)
{
    const double f = std::pow(10.0, decimals);
    return std::round(v * f) / f;
}
} // namespace

PathModel::PathModel(QObject *parent)
    : QAbstractListModel(parent)
{
}

int PathModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : static_cast<int>(m_points.size());
}

QVariant PathModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_points.size())
        return {};
    const QPointF &p = m_points.at(index.row());
    switch (role) {
    case XRole: return p.x();
    case YRole: return p.y();
    default: return {};
    }
}

QHash<int, QByteArray> PathModel::roleNames() const
{
    return { { XRole, "px" }, { YRole, "py" } };
}

void PathModel::setClosed(bool c)
{
    if (m_closed == c)
        return;
    m_closed = c;
    emit closedChanged();
}

void PathModel::setImageWidth(int w)
{
    if (m_imageWidth == w || w <= 0)
        return;
    m_imageWidth = w;
    emit imageSizeChanged();
}

void PathModel::setImageHeight(int h)
{
    if (m_imageHeight == h || h <= 0)
        return;
    m_imageHeight = h;
    emit imageSizeChanged();
}

QPointF PathModel::clamped(qreal x, qreal y) const
{
    return { std::clamp(x, 0.0, static_cast<qreal>(m_imageWidth)),
             std::clamp(y, 0.0, static_cast<qreal>(m_imageHeight)) };
}

void PathModel::addPoint(qreal x, qreal y)
{
    const int row = static_cast<int>(m_points.size());
    beginInsertRows({}, row, row);
    m_points.append(clamped(x, y));
    endInsertRows();
    emit pathChanged();
}

void PathModel::setPoint(int index, qreal x, qreal y)
{
    if (index < 0 || index >= m_points.size())
        return;
    m_points[index] = clamped(x, y);
    const QModelIndex mi = this->index(index);
    emit dataChanged(mi, mi, { XRole, YRole });
    emit pathChanged();
}

void PathModel::removeAt(int index)
{
    if (index < 0 || index >= m_points.size())
        return;
    beginRemoveRows({}, index, index);
    m_points.removeAt(index);
    endRemoveRows();
    emit pathChanged();
}

void PathModel::removeLast()
{
    removeAt(static_cast<int>(m_points.size()) - 1);
}

void PathModel::clear()
{
    if (m_points.isEmpty())
        return;
    beginResetModel();
    m_points.clear();
    endResetModel();
    emit pathChanged();
}

void PathModel::setPoints(const QVariantList &points)
{
    beginResetModel();
    m_points.clear();
    m_points.reserve(points.size());
    for (const QVariant &v : points) {
        const QVariantMap m = v.toMap();
        if (m.contains(QStringLiteral("x")) && m.contains(QStringLiteral("y")))
            m_points.append(clamped(m.value(QStringLiteral("x")).toDouble(),
                                    m.value(QStringLiteral("y")).toDouble()));
    }
    endResetModel();
    emit pathChanged();
}

QVariantList PathModel::points() const
{
    QVariantList list;
    list.reserve(m_points.size());
    for (const QPointF &p : m_points)
        list.append(QVariantMap{ { QStringLiteral("x"), p.x() },
                                 { QStringLiteral("y"), p.y() } });
    return list;
}

QRectF PathModel::boundingBox() const
{
    if (m_points.isEmpty())
        return {};
    qreal minX = std::numeric_limits<qreal>::max();
    qreal minY = std::numeric_limits<qreal>::max();
    qreal maxX = std::numeric_limits<qreal>::lowest();
    qreal maxY = std::numeric_limits<qreal>::lowest();
    for (const QPointF &p : m_points) {
        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
    }
    return { QPointF(minX, minY), QPointF(maxX, maxY) };
}

qreal PathModel::totalLength() const
{
    qreal len = 0;
    for (int i = 1; i < m_points.size(); ++i)
        len += QLineF(m_points[i - 1], m_points[i]).length();
    if (m_closed && m_points.size() > 2)
        len += QLineF(m_points.last(), m_points.first()).length();
    return len;
}

void PathModel::translateAll(qreal dx, qreal dy)
{
    if (m_points.isEmpty())
        return;
    const QRectF box = boundingBox();
    dx = std::clamp(dx, -box.left(), m_imageWidth - box.right());
    dy = std::clamp(dy, -box.top(), m_imageHeight - box.bottom());
    if (qFuzzyIsNull(dx) && qFuzzyIsNull(dy))
        return;
    for (QPointF &p : m_points)
        p += QPointF(dx, dy);
    emit dataChanged(index(0), index(static_cast<int>(m_points.size()) - 1), { XRole, YRole });
    emit pathChanged();
}

void PathModel::scaleAround(qreal cx, qreal cy, qreal factor)
{
    if (m_points.size() < 2)
        return;
    factor = std::clamp(factor, 0.05, 20.0);

    // 이미지 경계를 벗어나지 않는 최대 배율로 제한한다.
    for (const QPointF &p : m_points) {
        const qreal dx = p.x() - cx;
        const qreal dy = p.y() - cy;
        if (dx > 1e-6)
            factor = std::min(factor, (m_imageWidth - cx) / dx);
        else if (dx < -1e-6)
            factor = std::min(factor, cx / -dx);
        if (dy > 1e-6)
            factor = std::min(factor, (m_imageHeight - cy) / dy);
        else if (dy < -1e-6)
            factor = std::min(factor, cy / -dy);
    }
    if (factor <= 0 || qFuzzyCompare(factor, 1.0))
        return;

    for (QPointF &p : m_points)
        p = QPointF(cx + (p.x() - cx) * factor, cy + (p.y() - cy) * factor);
    emit dataChanged(index(0), index(static_cast<int>(m_points.size()) - 1), { XRole, YRole });
    emit pathChanged();
}

void PathModel::setError(const QString &msg)
{
    m_lastError = msg;
    emit lastErrorChanged();
}

QString PathModel::urlToLocalPath(const QUrl &url)
{
    if (url.isLocalFile())
        return url.toLocalFile();
    if (url.scheme().isEmpty())
        return url.toString();
    return {};
}

QUrl PathModel::tempFileUrl() const
{
    return QUrl::fromLocalFile(QDir::current().absoluteFilePath(QStringLiteral("path_temp.json")));
}

bool PathModel::exportToFile(const QUrl &url, const QVariantMap &options)
{
    const QString path = urlToLocalPath(url);
    if (path.isEmpty()) {
        setError(QStringLiteral("잘못된 파일 경로입니다."));
        return false;
    }
    if (m_points.size() < 2) {
        setError(QStringLiteral("경로 정점이 2개 이상 필요합니다."));
        return false;
    }

    const QString space = options.value(QStringLiteral("coordinateSpace"), QStringLiteral("pixel")).toString();
    const double workW = options.value(QStringLiteral("workAreaWidth"), 8.0).toDouble();
    const double workH = options.value(QStringLiteral("workAreaHeight"), 4.5).toDouble();
    const bool closedFlag = options.value(QStringLiteral("closed"), m_closed).toBool();
    const bool meters = (space == QStringLiteral("meter"));
    const double sx = meters ? workW / m_imageWidth : 1.0;
    const double sy = meters ? workH / m_imageHeight : 1.0;
    const int decimals = meters ? 4 : 1;

    QJsonArray pointsArray;
    for (const QPointF &p : m_points) {
        QJsonArray pair;
        pair.append(roundTo(p.x() * sx, decimals));
        pair.append(roundTo(p.y() * sy, decimals));
        pointsArray.append(pair);
    }

    QJsonObject payload;
    payload[QStringLiteral("points")] = pointsArray;

    QJsonObject meta;
    meta[QStringLiteral("app")] = QStringLiteral("Road-Painter QML Client");
    meta[QStringLiteral("schemaVersion")] = QStringLiteral("1.1");
    meta[QStringLiteral("createdAt")] = QDateTime::currentDateTime().toString(Qt::ISODate);
    meta[QStringLiteral("coordinateSpace")] = meters ? QStringLiteral("meter") : QStringLiteral("pixel");
    meta[QStringLiteral("imageSize")] = QJsonArray{ m_imageWidth, m_imageHeight };
    if (meters)
        meta[QStringLiteral("workAreaSize")] = QJsonArray{ workW, workH };
    meta[QStringLiteral("closed")] = closedFlag;
    meta[QStringLiteral("pointCount")] = static_cast<int>(m_points.size());

    QJsonObject root;
    root[QStringLiteral("type")] = QStringLiteral("BLUEPRINT");
    root[QStringLiteral("seq")] = m_exportSeq;
    root[QStringLiteral("payload")] = payload;
    root[QStringLiteral("meta")] = meta;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        setError(QStringLiteral("파일을 열 수 없습니다: %1").arg(path));
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();

    ++m_exportSeq;
    setError({});
    return true;
}

bool PathModel::importFromFile(const QUrl &url)
{
    const QString path = urlToLocalPath(url);
    if (path.isEmpty()) {
        setError(QStringLiteral("잘못된 파일 경로입니다."));
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setError(QStringLiteral("파일을 열 수 없습니다: %1").arg(path));
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    file.close();

    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        setError(QStringLiteral("JSON 파싱 실패: %1").arg(parseError.errorString()));
        return false;
    }

    const QJsonObject root = doc.object();
    QList<QPointF> loaded;
    bool closedFlag = false;

    if (root.contains(QStringLiteral("payload"))) {
        // BLUEPRINT 스키마: payload.points = [[x, y], ...]
        const QJsonArray pts = root.value(QStringLiteral("payload")).toObject()
                                   .value(QStringLiteral("points")).toArray();
        double sx = 1.0, sy = 1.0;
        const QJsonObject meta = root.value(QStringLiteral("meta")).toObject();
        if (meta.value(QStringLiteral("coordinateSpace")).toString() == QStringLiteral("meter")) {
            const QJsonArray work = meta.value(QStringLiteral("workAreaSize")).toArray();
            const double workW = work.size() == 2 ? work.at(0).toDouble(8.0) : 8.0;
            const double workH = work.size() == 2 ? work.at(1).toDouble(4.5) : 4.5;
            if (workW > 0 && workH > 0) {
                sx = m_imageWidth / workW;
                sy = m_imageHeight / workH;
            }
        }
        for (const QJsonValue &v : pts) {
            const QJsonArray pair = v.toArray();
            if (pair.size() >= 2)
                loaded.append(clamped(pair.at(0).toDouble() * sx, pair.at(1).toDouble() * sy));
        }
        closedFlag = meta.value(QStringLiteral("closed")).toBool(false);
    } else if (root.contains(QStringLiteral("path"))) {
        // 구버전(레거시) 스키마: path = [{sequence, x, y}, ...]
        const QJsonArray pts = root.value(QStringLiteral("path")).toArray();
        for (const QJsonValue &v : pts) {
            const QJsonObject o = v.toObject();
            loaded.append(clamped(o.value(QStringLiteral("x")).toDouble(),
                                  o.value(QStringLiteral("y")).toDouble()));
        }
    } else {
        setError(QStringLiteral("지원하지 않는 JSON 형식입니다. (payload.points 또는 path 필요)"));
        return false;
    }

    if (loaded.size() < 2) {
        setError(QStringLiteral("파일에 유효한 정점이 2개 이상 없습니다."));
        return false;
    }

    beginResetModel();
    m_points = loaded;
    endResetModel();
    setClosed(closedFlag);
    emit pathChanged();
    setError({});
    return true;
}

#ifndef PATHMODEL_H
#define PATHMODEL_H

#include <QAbstractListModel>
#include <QPointF>
#include <QRectF>
#include <QUrl>
#include <QVariantList>
#include <QVariantMap>
#include <QtQml/qqmlregistration.h>

// 경로(waypoint) 데이터 모델.
// 좌표는 목업 스트림의 이미지 픽셀 좌표계(기본 1280x720)를 사용하며,
// JSON 직렬화(BLUEPRINT 스키마)와 역직렬화를 담당한다.
class PathModel : public QAbstractListModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int count READ count NOTIFY pathChanged)
    Q_PROPERTY(bool closed READ closed WRITE setClosed NOTIFY closedChanged)
    Q_PROPERTY(int imageWidth READ imageWidth WRITE setImageWidth NOTIFY imageSizeChanged)
    Q_PROPERTY(int imageHeight READ imageHeight WRITE setImageHeight NOTIFY imageSizeChanged)
    Q_PROPERTY(QString lastError READ lastError NOTIFY lastErrorChanged)

public:
    enum Roles { XRole = Qt::UserRole + 1, YRole };

    explicit PathModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    int count() const { return static_cast<int>(m_points.size()); }
    bool closed() const { return m_closed; }
    void setClosed(bool c);
    int imageWidth() const { return m_imageWidth; }
    void setImageWidth(int w);
    int imageHeight() const { return m_imageHeight; }
    void setImageHeight(int h);
    QString lastError() const { return m_lastError; }

    Q_INVOKABLE void addPoint(qreal x, qreal y);
    Q_INVOKABLE void setPoint(int index, qreal x, qreal y);
    Q_INVOKABLE void removeAt(int index);
    Q_INVOKABLE void removeLast();
    Q_INVOKABLE void clear();
    Q_INVOKABLE void setPoints(const QVariantList &points);
    Q_INVOKABLE QVariantList points() const;
    Q_INVOKABLE QRectF boundingBox() const;
    Q_INVOKABLE qreal totalLength() const;
    Q_INVOKABLE void translateAll(qreal dx, qreal dy);
    Q_INVOKABLE void scaleAround(qreal cx, qreal cy, qreal factor);

    // options: closed(bool), coordinateSpace("pixel"|"meter"),
    //          workAreaWidth(m), workAreaHeight(m)
    Q_INVOKABLE bool exportToFile(const QUrl &url, const QVariantMap &options);
    Q_INVOKABLE bool importFromFile(const QUrl &url);
    Q_INVOKABLE QUrl tempFileUrl() const;

signals:
    void pathChanged();
    void closedChanged();
    void imageSizeChanged();
    void lastErrorChanged();

private:
    QPointF clamped(qreal x, qreal y) const;
    void setError(const QString &msg);
    static QString urlToLocalPath(const QUrl &url);

    QList<QPointF> m_points;
    bool m_closed = false;
    int m_imageWidth = 1280;
    int m_imageHeight = 720;
    QString m_lastError;
    int m_exportSeq = 1;
};

#endif // PATHMODEL_H

#ifndef VIDEOWIDGET_H
#define VIDEOWIDGET_H

#include <QLabel>
#include <QMouseEvent>
#include <QPoint>
#include <QVector>

enum class InteractionMode {
    Normal,
    MeasureCoordinate,
    DrawLinePath
};

class VideoWidget: public QLabel {
    Q_OBJECT
public:
    explicit VideoWidget(QWidget *parent = nullptr);
    void setInteractionMode(InteractionMode mode);
    void updateBackground(const QImage &image);
    void clearPath();


signals:
    void mousePositionChanged(int x, int y);
    // 💡 경로 지정이 완료되었을 때 최종 좌표 리스트를 담아 던져줄 시그널 추가
    void pathDrawingFinished(const QList<QPoint>& pathPoints);

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void leaveEvent(QEvent *event) override;

private:
    InteractionMode m_currentMode;
    QImage m_currentFrame;
    QPoint m_currentMousePos;
    bool m_mouseInWidget;

    // 경로 지정을 위한 변수들
    QVector<QPoint> m_pathPoints;

    // 위젯 크기 기준 마우스 좌표를 원본 영상 이미지 좌표계로 변환하는 헬퍼 함수
    QPoint mapToImageCoordinates(const QPoint &widgetPos);
};

#endif // VIDEOWIDGET_H

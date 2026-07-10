#include "videowidget.h"
#include <QPainter>
#include <QPen>
#include <QMouseEvent>
#include <QGuiApplication>
#include <cmath> // std::abs 사용을 위해 추가

VideoWidget::VideoWidget(QWidget *parent)
    : QLabel(parent), m_currentMode(InteractionMode::Normal), m_mouseInWidget(false) {
    setMouseTracking(true); // 마우스 클릭 없이도 move 이벤트를 받기 위해 필수 설정
    setAlignment(Qt::AlignCenter); //
}

void VideoWidget::setInteractionMode(InteractionMode mode) {
    m_currentMode = mode; //
    if (m_currentMode == InteractionMode::DrawLinePath) {
        clearPath(); // 일반 모드로 돌아오면 그려진 경로 초기화
    }
    update(); //
}

void VideoWidget::updateBackground(const QImage &image) {
    m_currentFrame = image; //
    update(); // 화면 재시각화 트리거 -> paintEvent 호출됨
}

void VideoWidget::clearPath() {
    m_pathPoints.clear(); //
}

void VideoWidget::mouseMoveEvent(QMouseEvent *event) {
    m_currentMousePos = event->pos(); //
    m_mouseInWidget = true; //

    if (m_currentMode == InteractionMode::MeasureCoordinate) {
        QPoint imgPos = mapToImageCoordinates(event->pos()); //
        emit mousePositionChanged(imgPos.x(), imgPos.y()); //
    }

    // 경로 지정 모드일 때 마우스가 움직이면 실시간 선 그리기를 위해 화면 갱신
    if (m_currentMode == InteractionMode::DrawLinePath) {
        update();
    }

    QLabel::mouseMoveEvent(event); //
}

void VideoWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) { //
        if (m_currentMode == InteractionMode::DrawLinePath) { //
            // 클릭된 좌표를 이미지 좌표계 기준으로 먼저 계산
            QPoint imgPos = mapToImageCoordinates(event->pos());

            // [Ctrl 기능 반영] 점이 최소 1개 이상 있고, Ctrl 키가 눌려있다면 좌표 보정 수행
            if (!m_pathPoints.isEmpty() && (event->modifiers() & Qt::ControlModifier)) {
                QPoint lastImgPos = m_pathPoints.last();
                int dx = std::abs(imgPos.x() - lastImgPos.x());
                int dy = std::abs(imgPos.y() - lastImgPos.y());

                if (dx > dy) {
                    imgPos.setY(lastImgPos.y()); // 수평선 스냅
                } else {
                    imgPos.setX(lastImgPos.x()); // 수직선 스냅
                }
            }

            m_pathPoints.append(imgPos); // 보정 완료된 좌표 저장
            update(); //
        }
    }
    // 💡 [핵심 추가] 오른쪽 마우스 클릭: 경로 지정 최종 완료 및 종료
    // =======================================================
    else if (event->button() == Qt::RightButton) {
        if (m_currentMode == InteractionMode::DrawLinePath) {
            // 최소 1개 이상의 포인트가 입력되었을 때만 완료 처리
            if (!m_pathPoints.isEmpty()) {
                // 상호작용 모드를 일반(Normal) 모드로 전환하여 노란색 가이드선과 마우스 추적을 종료
                m_currentMode = InteractionMode::Normal;

                // 💡 [핵심 추가] 경로가 완성되었으므로 MainWindow 측으로 좌표 리스트를 송신
                emit pathDrawingFinished(m_pathPoints);

                update(); // 노란 가이드선 제거 및 최종 선 확정을 위해 화면 갱신
            }
        }
    }

    QLabel::mousePressEvent(event); //
}

void VideoWidget::leaveEvent(QEvent *event) {
    m_mouseInWidget = false;
    update();
    QLabel::leaveEvent(event);
}

void VideoWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event); //
    QPainter painter(this); //

    if (m_currentFrame.isNull()) { //
        painter.fillRect(rect(), Qt::darkGray); //
        painter.setPen(Qt::white); //
        painter.drawText(rect(), Qt::AlignCenter, "CCTV 스트림 대기 중..."); //
        return; //
    }

    // 1. 비율 유지하며 이미지 스케일링 렌더링
    QPixmap pixmap = QPixmap::fromImage(m_currentFrame); //
    QPixmap scaledPixmap = pixmap.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation); //

    // 이미지가 중앙에 배치될 때의 여백(Offset) 계산
    int xOffset = (width() - scaledPixmap.width()) / 2; //
    int yOffset = (height() - scaledPixmap.height()) / 2; //
    painter.drawPixmap(xOffset, yOffset, scaledPixmap); //

    // 스케일 비율 계산 (좌표 역산용)
    double scaleX = (double)scaledPixmap.width() / m_currentFrame.width(); //
    double scaleY = (double)scaledPixmap.height() / m_currentFrame.height(); //

    // 2. 경로 지정 (직선) 그리기
    // 2. 경로 지정 (직선) 그리기
    if (m_currentMode == InteractionMode::DrawLinePath) {
        QPen linePen(Qt::red, 3); //
        painter.setPen(linePen); //

        // 이미 저장 완료된 고정 경로선, 번호 및 좌표 그리기
        for (int i = 0; i < m_pathPoints.size(); ++i) { //
            // 이미지 좌표 -> 현재 위젯 해상도 크기 좌표로 복원
            QPoint p1(m_pathPoints[i].x() * scaleX + xOffset, m_pathPoints[i].y() * scaleY + yOffset); //

            // 점 그리기
            painter.setBrush(Qt::red); //
            painter.setPen(QPen(Qt::red, 3));
            painter.drawEllipse(p1, 5, 5); //

            // 이전 점과 선 연결
            if (i > 0) { //
                QPoint p2(m_pathPoints[i-1].x() * scaleX + xOffset, m_pathPoints[i-1].y() * scaleY + yOffset); //
                painter.drawLine(p1, p2); //
            }

            // =======================================================
            // [💡 발전형 추가] 포인트 위에 번호와 좌표(X, Y) 표시 로직
            // =======================================================
            // 출력 형태 예시: "1. (145, 280)"
            QString infoText = QString("%1. (%2, %3)")
                                   .arg(i + 1)
                                   .arg(m_pathPoints[i].x())
                                   .arg(m_pathPoints[i].y());

            painter.setFont(QFont("Malgun Gothic", 10, QFont::Bold));

            // 좌표가 들어가면서 텍스트가 길어졌으므로 가로 폭(Width)을 120으로 넉넉하게 확장
            // 숫자가 점 바로 위에 겹치지 않도록 위쪽으로 25픽셀 Offset 조정
            QRect textRect(p1.x() - 60, p1.y() - 25, 120, 20);

            // 가독성을 위한 글자 외곽선(그림자) 효과 생성 (사방에 흰색 글씨 배치)
            painter.setPen(Qt::white);
            textRect.translate(-1, -1); painter.drawText(textRect, Qt::AlignCenter, infoText);
            textRect.translate(2, 0);   painter.drawText(textRect, Qt::AlignCenter, infoText);
            textRect.translate(0, 2);   painter.drawText(textRect, Qt::AlignCenter, infoText);
            textRect.translate(-2, 0);  painter.drawText(textRect, Qt::AlignCenter, infoText);

            // 원래 위치에 메인 텍스트 그리기 (빨간색 번호+좌표 표기)
            textRect.translate(1, -1); // 위치 원상 복구
            painter.setPen(Qt::red);
            painter.drawText(textRect, Qt::AlignCenter, infoText);
        }

        // [실시간 러버밴드 가이드선 기능]
        if (!m_pathPoints.isEmpty() && m_mouseInWidget) {
            QPoint lastImgPos = m_pathPoints.last();
            QPoint currentImgPos = mapToImageCoordinates(m_currentMousePos);

            // Ctrl 키 상태 확인 후 가이드선 좌표 보정
            if (QGuiApplication::keyboardModifiers() & Qt::ControlModifier) {
                int dx = std::abs(currentImgPos.x() - lastImgPos.x());
                int dy = std::abs(currentImgPos.y() - lastImgPos.y());
                if (dx > dy) {
                    currentImgPos.setY(lastImgPos.y());
                } else {
                    currentImgPos.setX(lastImgPos.x());
                }
            }

            // 위젯 화면 좌표계로 변환하여 점선 가이드 렌더링
            QPoint pLast(lastImgPos.x() * scaleX + xOffset, lastImgPos.y() * scaleY + yOffset);
            QPoint pCurrent(currentImgPos.x() * scaleX + xOffset, currentImgPos.y() * scaleY + yOffset);

            QPen guidePen(Qt::yellow, 2, Qt::DashLine);
            painter.setPen(guidePen);
            painter.drawLine(pLast, pCurrent);

            painter.setBrush(Qt::yellow);
            painter.setPen(QPen(Qt::yellow, 2));
            painter.drawEllipse(pCurrent, 4, 4);

            // [💡 가이드 기능 추가] 마우스를 따라다니는 실시간 다음 번호 및 좌표 미리보기
            QString nextInfoText = QString("%1. (%2, %3)")
                                       .arg(m_pathPoints.size() + 1)
                                       .arg(currentImgPos.x())
                                       .arg(currentImgPos.y());

            QRect guideTextRect(pCurrent.x() - 60, pCurrent.y() - 25, 120, 20);
            painter.setFont(QFont("Malgun Gothic", 9, QFont::Bold));

            // 노란 가이드 텍스트 외곽선 효과 (검은색 배경에 흰색 글씨로 눈에 띄게 처리)
            painter.setPen(Qt::black);
            guideTextRect.translate(-1, -1); painter.drawText(guideTextRect, Qt::AlignCenter, nextInfoText);
            guideTextRect.translate(2, 0);   painter.drawText(guideTextRect, Qt::AlignCenter, nextInfoText);
            guideTextRect.translate(0, 2);   painter.drawText(guideTextRect, Qt::AlignCenter, nextInfoText);
            guideTextRect.translate(-2, 0);  painter.drawText(guideTextRect, Qt::AlignCenter, nextInfoText);

            guideTextRect.translate(1, -1);
            painter.setPen(Qt::white);
            painter.drawText(guideTextRect, Qt::AlignCenter, nextInfoText);
        }
    }

    // 3. 좌표 측정 기능 활성화 시 커서 옆에 텍스트 표기
    if (m_currentMode == InteractionMode::MeasureCoordinate && m_mouseInWidget) { //
        QPoint imgPos = mapToImageCoordinates(m_currentMousePos); //
        QString coordText = QString("X:%1, Y:%2").arg(imgPos.x()).arg(imgPos.y()); //

        painter.setPen(Qt::black); //
        painter.setBrush(QColor(255, 255, 255, 180)); // 반투명 흰색 배경 사각형
        painter.drawRect(m_currentMousePos.x() + 15, m_currentMousePos.y() + 15, 100, 25); //

        painter.setPen(Qt::blue); //
        painter.setFont(QFont("Malgun Gothic", 10, QFont::Bold)); //
        painter.drawText(m_currentMousePos.x() + 20, m_currentMousePos.y() + 33, coordText); //
    }
}

QPoint VideoWidget::mapToImageCoordinates(const QPoint &widgetPos) {
    if (m_currentFrame.isNull()) return QPoint(0, 0); //

    QPixmap pixmap = QPixmap::fromImage(m_currentFrame); //
    QPixmap scaledPixmap = pixmap.scaled(size(), Qt::KeepAspectRatio); //

    int xOffset = (width() - scaledPixmap.width()) / 2; //
    int yOffset = (height() - scaledPixmap.height()) / 2; //

    int imgX = (widgetPos.x() - xOffset) * m_currentFrame.width() / scaledPixmap.width(); //
    int imgY = (widgetPos.y() - yOffset) * m_currentFrame.height() / scaledPixmap.height(); //

    // 범위 제한 경계 처리
    imgX = qBound(0, imgX, m_currentFrame.width() - 1); //
    imgY = qBound(0, imgY, m_currentFrame.height() - 1); //

    return QPoint(imgX, imgY); //
}
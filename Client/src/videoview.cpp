#include "videoview.h"
#include "motionprogram.h"

#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QCursor>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QHoverEvent>
#include <QGuiApplication>
#include <QStyleHints>
#include <QTimer>
#include <QJsonArray>
#include <QPolygonF>
#include <QLineF>
#include <QFont>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
// 작도 점 병합 기준: 화면에서 이만큼 가깝거나, 실제 거리로 이만큼 가까우면 같은 점으로 본다.
constexpr double kMergeScreenPx = 14.0;
constexpr double kMergeWorldMm  = 25.0;

// 격자/스케일바에 쓰는 "보기 좋은" 눈금 후보(mm)
const double kNiceSteps[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000 };

// ── 색 규칙 ──────────────────────────────────────────────────────────
// 실제로 칠할 페인트가 흰색이므로 "그려지는 것"은 전부 흰색으로 통일한다.
// 데모 바닥이 초록색이라 초록 계열은 쓰지 않는다.
//   흰색      = 그려질 선 / 도포 폭(밴드)  ← 결과물 그대로
//   흐린 흰색 = 이미 완성해둔 다른 도형
//   시안      = UI 표시용 (선택·핸들·스냅) — 페인트가 아니라 도구 색
//   주황      = 도색 완료된 구간(진행률)
const QColor kActive(255, 255, 255);        // 편집 중 경로 = 흰색
const QColor kDone(226, 232, 240, 175);     // 완성 도형 = 흐린 흰색
const QColor kSel(0, 229, 255);             // 선택/핸들 (도구 색)
const QColor kProgress(240, 112, 32);       // #F07020
const QColor kPaint(255, 255, 255);

// 점이 이보다 많으면 라벨/번호/배지를 접는다 (글자 도장처럼 촘촘한 경로)
constexpr int kDenseLimit = 28;

struct DisplayArc {
    bool ok = false;
    QPointF center;
    double radiusPx = 0.0;
    double sweepDeg = 0.0;
    bool left = true;
};

// 화면의 샘플 점이 실제 전송 단계에서 ARC 하나가 되는지 같은 판정기로 확인한다.
// 이 경우 각 현의 길이와 꼭짓점 회전각은 로봇 명령이 아니므로 편집 라벨로 보이면 안 된다.
DisplayArc displayArcFor(const QVector<QPointF> &pts, bool closed)
{
    DisplayArc out;
    if (pts.size() < 5) return out;
    QList<QPointF> run;
    run.reserve(pts.size() + (closed ? 1 : 0));
    for (const QPointF &p : pts) run.append(p);
    if (closed) run.append(pts.first());

    motionprogram::detail::Circle fit;
    if (!motionprogram::detail::arcFits(run, 0, run.size() - 1,
                                        fit, out.sweepDeg, out.left))
        return out;
    out.ok = true;
    out.center = fit.c;
    out.radiusPx = fit.r;
    return out;
}

QString mmLabel(double mm)
{
    return (mm >= 1000.0)
        ? QString::number(mm / 1000.0, 'f', 2) + QStringLiteral(" m")
        : QString::number(mm, 'f', (mm >= 100.0 ? 0 : 1)) + QStringLiteral(" mm");
}

// ── 직선 병합 (Ramer–Douglas–Peucker) ────────────────────────────────
// 거의 일직선인 점들을 지워 "직진 + 회전"만 남긴다.
QVector<QPointF> rdpSimplify(const QVector<QPointF> &pts, double eps)
{
    if (pts.size() < 3 || eps <= 0.0) return pts;
    QVector<bool> keep(pts.size(), false);
    keep.first() = true;
    keep.last() = true;

    QList<QPair<int, int>> stack;
    stack.append({ 0, int(pts.size()) - 1 });
    while (!stack.isEmpty()) {
        const QPair<int, int> seg = stack.takeLast();
        const int s = seg.first, e = seg.second;
        if (e <= s + 1) continue;
        const QPointF a = pts[s], b = pts[e];
        const double dx = b.x() - a.x(), dy = b.y() - a.y();
        const double len = std::hypot(dx, dy);
        double maxD = -1.0;
        int idx = -1;
        for (int i = s + 1; i < e; ++i) {
            const double d = (len < 1e-9)
                ? QLineF(pts[i], a).length()
                : std::abs(dy * (pts[i].x() - a.x()) - dx * (pts[i].y() - a.y())) / len;
            if (d > maxD) { maxD = d; idx = i; }
        }
        if (maxD > eps && idx > 0) {
            keep[idx] = true;
            stack.append({ s, idx });
            stack.append({ idx, e });
        }
    }
    QVector<QPointF> out;
    for (int i = 0; i < pts.size(); ++i)
        if (keep[i]) out.append(pts[i]);
    return out;
}

// 곡선 구간은 각도 간격만 지켜 남기고, 나머지만 RDP 로 줄인다.
// eps: 직선 구간 허용오차(px) · arcTol: 원호 잔차 허용(px) · stepDeg: 곡선 샘플 간격
} // namespace

VideoView::VideoView(QQuickItem *parent)
    : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::AllButtons);
    setAcceptHoverEvents(true);
    setRenderTarget(QQuickPaintedItem::FramebufferObject);

    m_clickTimer = new QTimer(this);
    m_clickTimer->setSingleShot(true);
    connect(m_clickTimer, &QTimer::timeout, this, [this]() {
        if (!m_pendingClick) return;
        m_pendingClick = false;
        commitDrawPoint(m_pendingImg);
    });
}

void VideoView::setTopView(bool v)   { if (m_isTopView != v)   { m_isTopView = v;   emit topViewChanged(); update(); } }
void VideoView::setInteractive(bool v){ if (m_interactive != v) { m_interactive = v; emit interactiveChanged(); } }

void VideoView::setShowLabels(bool v)
{
    if (m_showLabels == v) return;
    m_showLabels = v;
    if (!v) {
        // 다시 그리기 전의 짧은 순간에도 숨긴 배지가 클릭되지 않게 한다.
        m_edgeLabelRects.clear();
        m_turnBadgeRects.clear();
        m_hoverEdge = -1;
    }
    emit showLabelsChanged();
    update();
}

void VideoView::setStrokeWidthMm(double mm)
{
    const double v = qBound(1.0, mm, 500.0);
    if (qFuzzyCompare(m_strokeMm, v)) return;
    m_strokeMm = v;
    emit strokeWidthChanged();
    update();
}

// ── 표시 변환 / 좌표 매핑 ─────────────────────────────────────────────
double VideoView::viewPadding() const
{
    if (!m_isTopView) return 6.0;
    const double base = 22.0;
    return std::min(base, std::min(width(), height()) * 0.06);
}

double VideoView::fitScale() const
{
    if (m_frame.isNull()) return 1.0;
    const double pad = viewPadding();
    const double iw = std::max(1.0, width() - 2.0 * pad);
    const double ih = std::max(1.0, height() - 2.0 * pad);
    return std::min(iw / m_frame.width(), ih / m_frame.height());
}

void VideoView::displayTransform(double &sx, double &sy, double &ox, double &oy) const
{
    if (m_frame.isNull()) { sx = sy = 1.0; ox = oy = 0.0; return; }
    const double s = fitScale() * m_zoom;
    sx = sy = s;
    ox = (width() - m_frame.width() * s) / 2.0 + m_panX;
    oy = (height() - m_frame.height() * s) / 2.0 + m_panY;
}

void VideoView::clampPan()
{
    if (m_frame.isNull()) return;
    const double s = fitScale() * m_zoom;
    const double iw = m_frame.width() * s, ih = m_frame.height() * s;
    // 확대해도 작업면이 화면 밖으로 완전히 빠지지 않게
    const double margin = 90.0;
    const double maxX = std::max(0.0, (iw - width()) / 2.0) + margin;
    const double maxY = std::max(0.0, (ih - height()) / 2.0) + margin;
    m_panX = qBound(-maxX, m_panX, maxX);
    m_panY = qBound(-maxY, m_panY, maxY);
}

QPointF VideoView::mapToImage(const QPointF &pos) const
{
    if (m_frame.isNull()) return QPointF(0, 0);
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    if (sx <= 0.0) return QPointF(0, 0);
    double x = (pos.x() - ox) / sx;
    double y = (pos.y() - oy) / sy;
    x = qBound(0.0, x, double(m_frame.width() - 1));
    y = qBound(0.0, y, double(m_frame.height() - 1));
    return QPointF(x, y);
}

int VideoView::zoomPercent() const { return int(std::lround(m_zoom * 100.0)); }

void VideoView::zoomAtView(qreal vx, qreal vy, qreal factor)
{
    if (m_frame.isNull()) return;
    const double z = qBound(0.25, m_zoom * double(factor), 12.0);
    if (qFuzzyCompare(z, m_zoom)) return;

    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    const double imgX = (vx - ox) / sx;      // 커서 아래 이미지 좌표(클램프 없이)
    const double imgY = (vy - oy) / sy;

    m_zoom = z;
    const double s2 = fitScale() * m_zoom;
    m_panX = vx - imgX * s2 - (width() - m_frame.width() * s2) / 2.0;
    m_panY = vy - imgY * s2 - (height() - m_frame.height() * s2) / 2.0;
    clampPan();
    emit scaleChanged();
    update();
}

void VideoView::zoomBy(qreal factor)
{
    zoomAtView(width() / 2.0, height() / 2.0, factor);
}

// 배율 직접 입력 — 화면 중앙을 유지하며 그 배율로 맞춘다
void VideoView::setZoomPercent(int percent)
{
    const double want = qBound(25.0, double(percent), 1200.0) / 100.0;
    if (std::abs(want - m_zoom) < 1e-6) return;
    if (m_frame.isNull()) { m_zoom = want; emit scaleChanged(); update(); return; }
    zoomAtView(width() / 2.0, height() / 2.0, want / m_zoom);
}

// 드래그한 사각형이 화면을 채우도록 확대 (CAD 의 "윈도우 줌")
void VideoView::zoomToViewRect(qreal x, qreal y, qreal w, qreal h)
{
    if (m_frame.isNull() || w < 4 || h < 4) return;
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    if (sx <= 0) return;
    // 뷰 사각 → 이미지 사각
    const QRectF box((x - ox) / sx, (y - oy) / sy, w / sx, h / sy);
    const double base = fitScale();
    if (base <= 0) return;
    const double want = std::min(width() / box.width(), height() / box.height());
    m_zoom = qBound(0.25, want / base, 12.0);
    const double s = base * m_zoom;
    const QPointF c = box.center();
    m_panX = width() / 2.0 - c.x() * s - (width() - m_frame.width() * s) / 2.0;
    m_panY = height() / 2.0 - c.y() * s - (height() - m_frame.height() * s) / 2.0;
    clampPan();
    emit scaleChanged();
    update();
}

void VideoView::setZoomTool(bool on)
{
    if (m_zoomTool == on) return;
    m_zoomTool = on;
    setCursor(QCursor(on ? Qt::CrossCursor : Qt::ArrowCursor));
    emit zoomToolChanged();
    update();
}

void VideoView::fitView()
{
    m_zoom = 1.0;
    m_panX = m_panY = 0.0;
    emit scaleChanged();
    update();
}

void VideoView::zoomToSelection()
{
    const QList<int> t = transformTargets();
    if (t.size() < 2 || m_frame.isNull()) { fitView(); return; }
    QRectF box(m_points[t.first()], m_points[t.first()]);
    for (int i : t) {
        box.setLeft(std::min(box.left(), m_points[i].x()));
        box.setRight(std::max(box.right(), m_points[i].x()));
        box.setTop(std::min(box.top(), m_points[i].y()));
        box.setBottom(std::max(box.bottom(), m_points[i].y()));
    }
    const double pad = strokePx() + 24.0;
    box.adjust(-pad, -pad, pad, pad);
    if (box.width() < 1 || box.height() < 1) { fitView(); return; }

    const double base = fitScale();
    const double want = std::min(width() / box.width(), height() / box.height());
    m_zoom = qBound(0.25, want / base, 12.0);
    const double s = base * m_zoom;
    const QPointF c = box.center();
    m_panX = width() / 2.0 - c.x() * s - (width() - m_frame.width() * s) / 2.0;
    m_panY = height() / 2.0 - c.y() * s - (height() - m_frame.height() * s) / 2.0;
    clampPan();
    emit scaleChanged();
    update();
}

void VideoView::geometryChange(const QRectF &newGeom, const QRectF &oldGeom)
{
    QQuickPaintedItem::geometryChange(newGeom, oldGeom);
    if (newGeom.size() != oldGeom.size()) {
        clampPan();
        emit scaleChanged();
        update();
    }
}

double VideoView::screenPxPerMm() const
{
    if (m_tvPxPerM <= 1e-9 || m_frame.isNull()) return 0.0;
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    return m_tvPxPerM / 1000.0 * sx;
}

double VideoView::strokePx() const
{
    return (m_tvPxPerM > 1e-9) ? (m_strokeMm / 1000.0 * m_tvPxPerM) : 0.0;
}

// ── 프레임 수신 ───────────────────────────────────────────────────────
void VideoView::onFrame(const QImage &original)
{
    const QSize before = m_frame.size();
    if (m_isTopView) {
        // 🔴 여기 있던 80ms(≈12fps) 스로틀은 지웠다. 근거로 적혀 있던
        //    "1920x1080 을 통째로 펴서 GUI 스레드를 15~20ms 붙잡는다" 가 **사실이 아니다.**
        //    warpToTopView 의 출력은 TopView 캔버스 크기(840x560)라 입력 해상도와
        //    무관하다. 실측(2026-07-31, 1920x1080 입력):
        //        remap 0.86ms + QImage copy 0.38ms = 프레임당 1.23ms
        //    30fps 여도 GUI 스레드 부담은 초당 37ms(3.7%)뿐이다.
        //    반대로 스로틀 때문에 들어온 프레임을 버려서 TopView 만 끊겨 보였다.
        //    ⚠️ 다시 넣고 싶어지면 먼저 재라. 추정으로 넣은 값이 이 사달을 냈다.
        QImage w = warpToTopView(original);
        m_frame = w.isNull() ? original : w;
    } else {
        m_frame = original;
    }
    if (m_frame.size() != before)
        emit scaleChanged();
    update();
}

// ── 페인팅 ────────────────────────────────────────────────────────────
void VideoView::paint(QPainter *p)
{
    p->setRenderHint(QPainter::Antialiasing, true);
    p->setRenderHint(QPainter::SmoothPixmapTransform, true);

    if (m_frame.isNull()) {
        p->fillRect(boundingRect(), QColor("#1A1D21"));
        p->setPen(QColor("#8B939C"));
        p->drawText(boundingRect(), Qt::AlignCenter,
                    m_isTopView ? "TopView 대기" : "CCTV 대기");
        return;
    }

    p->fillRect(boundingRect(), QColor("#1A1D21"));

    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    QRectF target(ox, oy, m_frame.width() * sx, m_frame.height() * sy);
    p->save();
    p->setClipRect(boundingRect());
    p->drawImage(target, m_frame);

    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(QColor(255, 255, 255, 55), 1));
    p->drawRect(target.adjusted(-0.5, -0.5, 0.5, 0.5));

    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };

    if (m_isTopView)
        paintGrid(p, sx, sy, ox, oy);

    // 0) 미션 ghost + progress fill (TopView / CCTV 공통)
    if (!m_missionPaths.isEmpty())
        paintMission(p, sx, sy, ox, oy);

    // 1) 원본 뷰(CCTV): 편집 중 경로는 "선만" — 꼭짓점 점은 표시하지 않는다.
    //    실제 도포 폭 밴드 + 중심선으로 어디에 어떻게 칠할지만 보여준다.
    if (!m_isTopView && !m_overlayPaths.isEmpty() && m_missionPaths.isEmpty()) {
        // 실제 도포 폭 밴드. TopView 에서 외곽선을 만들어 원근+렌즈 왜곡까지
        // 태워 온 폴리곤이라, 원본 영상 위 실제 칠할 자리와 그대로 맞는다.
        // 바깥 고리와 안쪽 고리가 같이 오므로 OddEven 으로 채워야 도넛이 된다.
        if (!m_overlayBands.isEmpty()) {
            QPainterPath bandPath;
            bandPath.setFillRule(Qt::OddEvenFill);
            for (const QPolygonF &ring : std::as_const(m_overlayBands)) {
                QPolygonF wr;
                wr.reserve(ring.size());
                for (const QPointF &ip : ring) wr.append(toW(ip));
                bandPath.addPolygon(wr);
            }
            p->setPen(Qt::NoPen);
            p->fillPath(bandPath, QColor(255, 255, 255, 80));
        }
        for (const VVPath &op : std::as_const(m_overlayPaths)) {
            if (op.pts.size() < 2) continue;
            p->setPen(QPen(kActive, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p->setBrush(Qt::NoBrush);
            for (int i = 1; i < op.pts.size(); ++i)
                p->drawLine(toW(op.pts[i - 1]), toW(op.pts[i]));
            if (op.closed && op.pts.size() > 2)
                p->drawLine(toW(op.pts.last()), toW(op.pts.first()));
        }
    }

    // 2) TopView: 완성 도형들 + 활성 경로
    if (m_isTopView) {
        paintDonePaths(p, sx, sy, ox, oy);

        if (m_points.isEmpty()) {
            m_edgeLabelRects.clear();
            m_turnBadgeRects.clear();
        }

        if (!m_points.isEmpty()) {
            const bool dense = m_points.size() > kDenseLimit;
            const DisplayArc displayArc = displayArcFor(m_points, m_closed);

            const double arcRadiusM = displayArc.ok
                                    ? displayArc.radiusPx / std::max(1e-9, m_tvPxPerM)
                                    : 0.0;
            const bool arcTooTight = displayArc.ok
                                  && arcRadiusM + 1e-9
                                     < motionprogram::kServerConfirmedMinPaintRadiusM;

            // 실제 도포 폭(50 mm) — 불가능한 ARC는 도면에서 바로 빨간색으로 보인다.
            paintBand(p, m_points, m_closed, sx, sy, ox, oy,
                      arcTooTight ? QColor(229, 107, 107, 95)
                                  : QColor(255, 255, 255, 80));

            p->setPen(QPen(arcTooTight ? QColor(229, 107, 107) : kActive,
                           2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p->setBrush(Qt::NoBrush);
            for (int i = 1; i < m_points.size(); ++i)
                p->drawLine(toW(m_points[i - 1]), toW(m_points[i]));
            if (m_closed && m_points.size() > 2)
                p->drawLine(toW(m_points.last()), toW(m_points.first()));

            // 편집 표시가 꺼져 있으면(빈 곳 클릭) 완성 도형과 똑같이 그린다.
            const bool edit = m_focused || m_drawing;
            if (edit && m_showLabels && !dense && !displayArc.ok) {
                paintEdgeLengths(p, m_points, m_closed, sx, sy, ox, oy);
            } else {
                m_edgeLabelRects.clear();
                m_turnBadgeRects.clear();
            }
            paintPathGuides(p, m_points, m_closed, sx, sy, ox, oy,
                            edit && m_showLabels && !dense && !displayArc.ok);

            // 작도 중 첫 점은 "여기 클릭하면 닫힘" 표시
            if (m_drawing && m_points.size() >= 3 && !m_closed) {
                p->setBrush(Qt::NoBrush);
                p->setPen(QPen(kSel, 1.5, Qt::DashLine));
                p->drawEllipse(toW(m_points.first()), 11, 11);
            }

            // 점 핸들 — 선은 흰색(=페인트), 점은 시안(=도구)이라 서로 구분된다
            p->setFont(QFont("Pretendard", 9, QFont::Bold));
            for (int i = 0; i < m_points.size(); ++i) {
                const QPointF w = toW(m_points[i]);
                const bool sel = m_selection.contains(i);
                if (!edit) {
                    // 완성 도형과 같은 작은 점만. S/E·번호도 붙이지 않는다.
                    p->setBrush(sel ? QColor(Qt::white) : kDone);
                    p->setPen(QPen(sel ? kSel : QColor(26, 29, 33, 180), sel ? 2.0 : 1.0));
                    p->drawEllipse(w, sel ? 5.0 : 2.6, sel ? 5.0 : 2.6);
                    continue;
                }
                const bool hot = (i == m_hoverVertex);
                const bool isStart = (i == 0);
                const bool isEnd = (!m_closed && i == m_points.size() - 1 && m_points.size() > 1);
                const double r = sel ? 6.0 : (dense ? 2.6 : 4.5);
                p->setBrush(sel ? QColor(Qt::white) : (isStart ? kSel.lighter(135) : kSel));
                p->setPen(QPen(sel ? kSel : (hot ? QColor(Qt::white) : QColor(26, 29, 33, 200)),
                               sel ? 2.5 : 1.4));
                p->drawEllipse(w, r, r);

                if (!dense) {
                    const QString t = isStart ? QStringLiteral("S")
                                    : (isEnd ? QStringLiteral("E") : QString::number(i + 1));
                    const QRectF tr(w.x() - 20, w.y() - 24, 40, 16);
                    p->setPen(QColor(0, 0, 0, 170));  p->drawText(tr.translated(1, 1), Qt::AlignCenter, t);
                    p->setPen(Qt::white);  p->drawText(tr, Qt::AlignCenter, t);
                }
            }

            // 다음 점 미리보기 (작도 중)
            if (m_drawing && m_hover && !m_closed) {
                const QPointF last = m_points.last();
                QPointF cur = mapToImage(m_mouse);
                const bool ctrl = QGuiApplication::keyboardModifiers() & Qt::ControlModifier;

                if (ctrl) {
                    const bool horiz = std::abs(cur.x() - last.x()) > std::abs(cur.y() - last.y());
                    if (horiz) cur.setY(last.y()); else cur.setX(last.x());
                    QPointF a, b;
                    if (horiz) { a = QPointF(0, last.y()); b = QPointF(m_frame.width() - 1, last.y()); }
                    else       { a = QPointF(last.x(), 0); b = QPointF(last.x(), m_frame.height() - 1); }
                    p->setPen(QPen(QColor(255, 255, 255, 100), 1, Qt::DotLine));
                    p->drawLine(toW(a), toW(b));
                }

                const double thr = mergeThresholdPx();
                int snap = -1; double best = thr;
                for (int i = 0; i < m_points.size(); ++i) {
                    const double d = QLineF(cur, m_points[i]).length();
                    if (d <= best) { best = d; snap = i; }
                }
                if (snap >= 0) cur = m_points[snap];

                // 미리보기도 실제 도포 폭으로 — 두께 감이 바로 온다
                const double wpx = strokePx() * sx;
                if (wpx >= 2.0) {
                    p->setPen(QPen(QColor(255, 255, 255, 55), wpx, Qt::SolidLine,
                                   Qt::RoundCap, Qt::RoundJoin));
                    p->drawLine(toW(last), toW(cur));
                }
                QPen preview(QColor(255, 255, 255, 190), 1.5, Qt::DashLine, Qt::RoundCap);
                preview.setDashPattern({4, 4});
                p->setPen(preview);
                p->drawLine(toW(last), toW(cur));

                p->setBrush(Qt::NoBrush);
                if (snap >= 0) {
                    const bool closes = (snap == 0 && m_points.size() >= 3);
                    p->setPen(QPen(closes ? kProgress : kSel, 2.5));
                    p->drawEllipse(toW(cur), 8, 8);
                } else {
                    p->setPen(QPen(QColor(255, 255, 255, 220), 1.5));
                    p->drawEllipse(toW(cur), 5, 5);
                }
            }

            // 원호는 실제 전송 단위와 같은 요약만 보여준다. 원 중심은 점 번호와
            // 마커가 가장 많이 겹치는 자리이므로 쓰지 않는다. 변형 안내가 선택
            // 박스 아래·왼쪽에 있으니 ARC 요약은 도형 위·오른쪽에 둔다. 점과
            // 번호를 다 그린 뒤 덮어 그려 배경 영상이 복잡해도 읽히게 한다.
            if (edit && m_showLabels && !dense && displayArc.ok) {
                const double radiusMm = displayArc.radiusPx
                                      / std::max(1e-9, m_tvPxPerM) * 1000.0;
                const QString tag = arcTooTight
                    ? QStringLiteral("도색 불가 · R %1 · 최소 200 mm").arg(mmLabel(radiusMm))
                    : QStringLiteral("ARC · R %1 · %2°")
                          .arg(mmLabel(radiusMm)).arg(displayArc.sweepDeg, 0, 'f', 0);
                p->setFont(QFont("Pretendard", 8));
                const QFontMetrics fm(p->font());
                QRectF pathBox;
                for (const QPointF &point : std::as_const(m_points)) {
                    const QPointF viewPoint(point.x() * sx + ox, point.y() * sy + oy);
                    pathBox |= QRectF(viewPoint, QSizeF(0.1, 0.1));
                }
                const double boxWidth = fm.horizontalAdvance(tag) + 12.0;
                const double boxHeight = fm.height() + 6.0;
                double boxX = pathBox.right() - boxWidth;
                double boxY = pathBox.top() - boxHeight - 7.0;
                if (boxY < 2.0) boxY = pathBox.top() + 7.0;
                QRectF box(boxX, boxY, boxWidth, boxHeight);
                box.moveLeft(qBound(2.0, box.left(),
                                    std::max(2.0, width() - box.width() - 2.0)));
                box.moveTop(qBound(2.0, box.top(),
                                   std::max(2.0, height() - box.height() - 2.0)));
                p->setPen(Qt::NoPen);
                p->setBrush(arcTooTight ? QColor(92, 24, 24, 225)
                                        : QColor(26, 29, 33, 210));
                p->drawRoundedRect(box, 3, 3);
                p->setPen(arcTooTight ? QColor(255, 190, 190)
                                      : QColor(kSel.red(), kSel.green(), kSel.blue(), 235));
                p->drawText(box, Qt::AlignCenter, tag);
            }
        }

        // 선택 영역 박스 + 크기조절/회전 핸들
        paintSelectionBox(p, sx, sy, ox, oy);

        // 올가미 선택 / 영역 확대 사각
        if (m_rubber) {
            const QRectF r = QRectF(m_rubberStart, m_rubberEnd).normalized();
            if (m_rubberZoom) {
                p->setPen(QPen(QColor(255, 214, 0), 1.4, Qt::DashLine));
                p->setBrush(QColor(255, 214, 0, 34));
                p->drawRect(r);
                p->setPen(QColor(255, 214, 0));
                p->setFont(QFont("Pretendard", 9, QFont::DemiBold));
                p->drawText(r.adjusted(6, 4, -6, -4), Qt::AlignLeft | Qt::AlignTop,
                            QStringLiteral("이 영역으로 확대"));
            } else {
                p->setPen(QPen(kSel, 1, Qt::DashLine));
                p->setBrush(QColor(0, 229, 255, 30));
                p->drawRect(r);
            }
        }
    }

    // 3) 로봇 마커 — 끄면 그리는 것만 멈춘다(위치 수신·진행률 계산은 그대로).
    if (m_isTopView && m_robotVisible)
        paintRobot(p, sx, sy, ox, oy);

    // 4) 원점 힌트 — 좌하단 (0,0)
    if (m_isTopView && m_hasMmToTv) {
        const QPointF w = toW(worldMmToTopPx(0, 0));
        p->setPen(QPen(QColor(255, 255, 255, 180), 1));
        p->drawLine(QPointF(w.x() - 8, w.y()), QPointF(w.x() + 14, w.y()));
        p->drawLine(QPointF(w.x(), w.y() - 14), QPointF(w.x(), w.y() + 8));
        p->setFont(QFont("Pretendard", 8));
        p->setPen(QColor(255, 255, 255, 200));
        p->drawText(QPointF(w.x() + 6, w.y() - 6), QStringLiteral("0,0"));
    }

    // ⚠️ 여기 있던 paintMarker(서버가 중계한 로봇 마커 원본 픽셀) 호출은 지웠다.
    //    POS 가 2026-07-27 부터 QT 로 오지 않아 setMarkerCorners 를 부르는 곳이
    //    사라졌고, m_markerPx 는 영원히 비어서 매 프레임 즉시 return 만 했다.
    //    로봇 마커 표시는 로컬 ArUco 검출(paintAruco)이 담당한다.

    // ArUco 마커 ID 오버레이 (양쪽 뷰)
    paintAruco(p, sx, sy, ox, oy);

    p->restore();
}

// 도장 폭 밴드 — 실제로 칠해질 면적
void VideoView::paintBand(QPainter *p, const QVector<QPointF> &pts, bool closed,
                          double sx, double sy, double ox, double oy, const QColor &c)
{
    // 도장 폭은 TopView(보정된 바닥) 축척에서만 의미가 있다.
    // CCTV 원본은 원근 때문에 폭이 위치마다 달라지므로 선만 그린다.
    if (!m_isTopView || pts.size() < 2) return;
    const double wpx = strokePx() * sx;
    if (wpx < 1.5) return;
    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };
    QPainterPath path;
    path.moveTo(toW(pts[0]));
    for (int i = 1; i < pts.size(); ++i)
        path.lineTo(toW(pts[i]));
    if (closed && pts.size() > 2)
        path.closeSubpath();
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(c, wpx, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->drawPath(path);
}

// 완성된 도형들 — 클릭하면 다시 활성화된다.
void VideoView::paintDonePaths(QPainter *p, double sx, double sy, double ox, double oy)
{
    if (m_done.isEmpty()) return;
    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };
    for (const VVPath &dp : std::as_const(m_done)) {
        if (dp.pts.size() < 2) continue;
        paintBand(p, dp.pts, dp.closed, sx, sy, ox, oy, QColor(255, 255, 255, 45));

        p->setPen(QPen(kDone, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->setBrush(Qt::NoBrush);
        for (int i = 1; i < dp.pts.size(); ++i)
            p->drawLine(toW(dp.pts[i - 1]), toW(dp.pts[i]));
        if (dp.closed && dp.pts.size() > 2)
            p->drawLine(toW(dp.pts.last()), toW(dp.pts.first()));

        if (dp.pts.size() <= kDenseLimit) {
            const int si = int(&dp - &m_done.constFirst());
            const QSet<int> sel = (si >= 0 && si < m_doneSel.size()) ? m_doneSel[si]
                                                                     : QSet<int>();
            for (int i = 0; i < dp.pts.size(); ++i) {
                const bool on = sel.contains(i);
                p->setBrush(on ? QColor(Qt::white) : kDone);
                p->setPen(QPen(on ? kSel : QColor(26, 29, 33, 180), on ? 2.0 : 1.0));
                p->drawEllipse(toW(dp.pts[i]), on ? 5.0 : 2.6, on ? 5.0 : 2.6);
            }
            paintPathGuides(p, dp.pts, dp.closed, sx, sy, ox, oy, false);
        }
    }
}

// 진행 방향 화살표 + 꼭짓점 회전 배지 + 시작/끝
void VideoView::paintPathGuides(QPainter *p, const QVector<QPointF> &pts, bool closed,
                                double sx, double sy, double ox, double oy, bool full)
{
    if (pts.size() < 2) return;
    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };

    const int nSeg = (closed && pts.size() > 2) ? pts.size() : pts.size() - 1;
    for (int i = 0; i < nSeg; ++i) {
        const QPointF a = toW(pts[i]);
        const QPointF b = toW(pts[(i + 1) % pts.size()]);
        if (QLineF(a, b).length() < 52) continue;
        drawSegArrow(p, a, b, full ? QColor(255, 255, 255, 235) : QColor(255, 255, 255, 150));
    }

    if (!full) return;

    // 꼭짓점 회전 배지 — 프로토콜과 동일하게 양수 = 좌회전.
    // 배지 위치를 기억해 두면 클릭으로 각도를 직접 고칠 수 있다.
    m_turnBadgeRects.fill(QRectF(), pts.size());
    const bool hoverable = (&pts == &m_points);
    p->setFont(QFont("Pretendard", 8, QFont::DemiBold));
    const QFontMetrics fm(p->font());
    const int first = closed ? 0 : 1;
    const int last  = closed ? pts.size() - 1 : pts.size() - 2;
    for (int i = first; i <= last && pts.size() >= 3; ++i) {
        const QPointF prev = pts[(i - 1 + pts.size()) % pts.size()];
        const QPointF cur  = pts[i];
        const QPointF next = pts[(i + 1) % pts.size()];
        const QPointF v1 = cur - prev;
        const QPointF v2 = next - cur;
        if (QLineF(toW(prev), toW(cur)).length() < 46.0
            || QLineF(toW(cur), toW(next)).length() < 46.0) continue;
        // 화면 y는 아래로 — 월드 회전 방향은 화면 외적의 부호 반전
        const double crossW = -(v1.x() * v2.y() - v1.y() * v2.x());
        const double dot = v1.x() * v2.x() + v1.y() * v2.y();
        const double deg = std::atan2(crossW, dot) * 180.0 / M_PI;
        if (std::abs(deg) < 3.0) continue;
        const QString label = (deg > 0 ? QStringLiteral("↺좌 ") : QStringLiteral("↻우 "))
                              + QString::number(std::abs(deg), 'f', 0) + QStringLiteral("°");
        const QPointF w = toW(cur);
        const QRect br = fm.boundingRect(label);
        QRectF box(w.x() + 9, w.y() + 9, br.width() + 10, br.height() + 4);
        if (box.right() > width() - 2) box.moveLeft(w.x() - box.width() - 9);
        if (box.bottom() > height() - 2) box.moveTop(w.y() - box.height() - 9);
        const bool hot = hoverable && box.contains(m_mouse);
        p->setPen(Qt::NoPen);
        p->setBrush(hot ? QColor(0, 229, 255, 235) : QColor(26, 29, 33, 205));
        p->drawRoundedRect(box, 3, 3);
        p->setPen(hot ? QColor(12, 16, 20)
                      : (deg > 0 ? QColor(120, 200, 255) : QColor(255, 190, 120)));
        p->drawText(box, Qt::AlignCenter, label);
        if (hoverable && i < m_turnBadgeRects.size()) m_turnBadgeRects[i] = box;
    }
}

void VideoView::drawSegArrow(QPainter *p, const QPointF &aW, const QPointF &bW,
                             const QColor &fill, double t)
{
    const QPointF pos = aW + (bW - aW) * t;
    const double ang = std::atan2(bW.y() - aW.y(), bW.x() - aW.x()) * 180.0 / M_PI;
    p->save();
    p->translate(pos);
    p->rotate(ang);
    QPolygonF tri;
    tri << QPointF(7, 0) << QPointF(-4, 4.6) << QPointF(-4, -4.6);
    p->setPen(QPen(QColor(26, 29, 33, 140), 1));
    p->setBrush(fill);
    p->drawPolygon(tri);
    p->restore();
}

// 100 mm 급 격자
void VideoView::paintGrid(QPainter *p, double sx, double sy, double ox, double oy)
{
    Q_UNUSED(sy);
    if (!m_hasMmToTv || m_frame.isNull()) return;
    const double pxPerMm = m_tvPxPerM / 1000.0;
    if (pxPerMm <= 1e-9 || sx <= 0.0) return;

    double stepMm = kNiceSteps[0];
    for (double s : kNiceSteps) { stepMm = s; if (s * pxPerMm * sx >= 26.0) break; }
    const double stepPx = stepMm * pxPerMm;
    if (stepPx <= 0.5) return;

    const QPointF origin = worldMmToTopPx(0, 0);
    const double fw = m_frame.width(), fh = m_frame.height();

    p->save();
    p->setClipRect(QRectF(ox, oy, fw * sx, fh * sx).intersected(boundingRect()));
    p->setPen(QPen(QColor(255, 255, 255, 26), 1));

    const double firstX = origin.x() - std::floor(origin.x() / stepPx) * stepPx;
    for (double x = firstX; x <= fw; x += stepPx) {
        const double vx = x * sx + ox;
        p->drawLine(QPointF(vx, oy), QPointF(vx, oy + fh * sx));
    }
    const double firstY = origin.y() - std::floor(origin.y() / stepPx) * stepPx;
    for (double y = firstY; y <= fh; y += stepPx) {
        const double vy = y * sx + oy;
        p->drawLine(QPointF(ox, vy), QPointF(ox + fw * sx, vy));
    }
    p->restore();
}

// ArUco 오버레이 — CCTV: 노란 외곽선 + ID 칩 / TopView: 변환한 위치에 ID 칩
void VideoView::paintAruco(QPainter *p, double sx, double sy, double ox, double oy)
{
    if (!m_arucoVisible || m_arucoPolys.isEmpty()) return;

    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };
    const QColor line(255, 197, 61);
    p->setFont(QFont("Pretendard", 8, QFont::Bold));
    const QFontMetrics fm(p->font());

    for (int i = 0; i < m_arucoPolys.size(); ++i) {
        QPolygonF poly = m_arucoPolys[i];
        if (poly.size() < 3) continue;

        if (m_isTopView) {
            if (!m_tvBuilt || m_tvH.empty()) return;
            std::vector<cv::Point2f> in, outPts;
            for (const QPointF &q : std::as_const(poly))
                in.emplace_back(float(q.x()), float(q.y()));
            cv::perspectiveTransform(in, outPts, m_tvH);
            poly.clear();
            for (const cv::Point2f &q : outPts) poly << QPointF(q.x, q.y);
            const QPointF c = poly.boundingRect().center();
            if (c.x() < -40 || c.y() < -40
                || c.x() > m_frame.width() + 40 || c.y() > m_frame.height() + 40)
                continue;
        }

        QPolygonF w;
        for (const QPointF &q : std::as_const(poly)) w << toW(q);

        if (!m_isTopView) {
            p->setBrush(QColor(255, 197, 61, 26));
            p->setPen(QPen(line, 1.6));
            p->drawPolygon(w);
            p->setBrush(line);
            p->setPen(Qt::NoPen);
            p->drawEllipse(w.first(), 2.5, 2.5);
        }

        const QString label = QStringLiteral("ID %1").arg(m_arucoIds.value(i, -1));
        const QRect br = fm.boundingRect(label);
        const QPointF c = w.boundingRect().center();
        QRectF box(c.x() - br.width() / 2.0 - 4, c.y() - br.height() / 2.0 - 2,
                   br.width() + 8, br.height() + 4);
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(26, 29, 33, 200));
        p->drawRoundedRect(box, 3, 3);
        p->setPen(line);
        p->drawText(box, Qt::AlignCenter, label);
    }
}

void VideoView::setArucoMarkers(const QList<int> &ids, const QList<QPolygonF> &cornersPx)
{
    m_arucoIds = ids;
    m_arucoPolys = cornersPx;
    update();
}

void VideoView::setArucoVisible(bool on)
{
    if (m_arucoVisible == on) return;
    m_arucoVisible = on;
    update();
}

void VideoView::setRobotVisible(bool on)
{
    if (m_robotVisible == on) return;
    m_robotVisible = on;
    update();
}

void VideoView::paintMission(QPainter *p, double sx, double sy, double ox, double oy)
{
    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };

    QList<QVector<QPointF>> segsList;
    double total = 0.0;
    for (const VVPath &mp : std::as_const(m_missionPaths)) {
        if (mp.pts.size() < 2) continue;
        QVector<QPointF> segs = mp.pts;
        if (mp.closed && mp.pts.size() > 2)
            segs.append(mp.pts.first());
        segsList.append(segs);
        for (int i = 1; i < segs.size(); ++i)
            total += QLineF(segs[i - 1], segs[i]).length();
    }
    if (segsList.isEmpty()) return;

    const double wpx = strokePx() * sx;

    // 아직 안 칠한 계획 — 흰 밴드(도포 폭) + 점선 중심선
    for (const QVector<QPointF> &segs : std::as_const(segsList)) {
        paintBand(p, segs, false, sx, sy, ox, oy, QColor(255, 255, 255, 55));
        QPen dash(QColor(255, 255, 255, 170), 2, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
        p->setPen(dash);
        p->setBrush(Qt::NoBrush);
        for (int i = 1; i < segs.size(); ++i)
            p->drawLine(toW(segs[i - 1]), toW(segs[i]));
        for (int i = 1; i < segs.size(); ++i) {
            const QPointF a = toW(segs[i - 1]);
            const QPointF b = toW(segs[i]);
            if (QLineF(a, b).length() >= 52)
                drawSegArrow(p, a, b, QColor(255, 255, 255, 170));
        }
    }

    if (total < 1e-6) return;

    // 칠해진 구간 — 실제 도포 폭으로
    double remain = total * qBound(0.0, m_missionProgress, 1.0);
    QPen fillPen(kProgress, std::max(6.0, wpx), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p->setPen(fillPen);
    p->setBrush(Qt::NoBrush);
    for (const QVector<QPointF> &segs : std::as_const(segsList)) {
        for (int i = 1; i < segs.size() && remain > 1e-9; ++i) {
            const QPointF a = segs[i - 1];
            const QPointF b = segs[i];
            const double segLen = QLineF(a, b).length();
            if (segLen < 1e-9) continue;
            if (remain >= segLen) {
                p->drawLine(toW(a), toW(b));
                remain -= segLen;
            } else {
                const double t = remain / segLen;
                p->drawLine(toW(a), toW(a + (b - a) * t));
                remain = 0;
            }
        }
        if (remain <= 1e-9) break;
    }
}

// 변 길이 라벨 — 활성 경로만. 클릭 편집용 히트박스를 기억한다.
void VideoView::paintEdgeLengths(QPainter *p, const QVector<QPointF> &pts, bool closed,
                                 double sx, double sy, double ox, double oy)
{
    m_edgeLabelRects.clear();
    if (pts.size() < 2) return;
    const double spm = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
    auto toW = [&](const QPointF &ip) {
        return QPointF(ip.x() * sx + ox, ip.y() * sy + oy);
    };

    p->setFont(QFont("Pretendard", 8, QFont::DemiBold));
    const QFontMetrics fm(p->font());
    const int nSeg = (closed && pts.size() > 2) ? pts.size() : pts.size() - 1;
    m_edgeLabelRects.resize(nSeg);

    for (int i = 0; i < nSeg; ++i) {
        const QPointF &a = pts[i];
        const QPointF &b = pts[(i + 1) % pts.size()];
        const QPointF wa = toW(a), wb = toW(b);
        // 화면에서 너무 짧은 변은 라벨 생략 — 겹쳐서 못 읽는 걸 방지
        if (QLineF(wa, wb).length() < 44.0) { m_edgeLabelRects[i] = QRectF(); continue; }
        const double mm = QLineF(a, b).length() / spm * 1000.0;
        if (mm < 0.5) { m_edgeLabelRects[i] = QRectF(); continue; }
        const QPointF mid = (wa + wb) * 0.5;
        const QString label = mmLabel(mm);
        const QRect br = fm.boundingRect(label);
        QRectF box(mid.x() - br.width() * 0.5 - 5, mid.y() - br.height() * 0.5 - 3,
                   br.width() + 10, br.height() + 6);

        const double mx = 2.0;
        box.moveLeft(qBound(mx, box.left(), std::max(mx, width() - box.width() - mx)));
        box.moveTop(qBound(mx, box.top(), std::max(mx, height() - box.height() - mx)));
        m_edgeLabelRects[i] = box;

        const bool hot = (i == m_hoverEdge) && m_interactive && !m_drawing;
        p->setPen(Qt::NoPen);
        p->setBrush(hot ? QColor(0, 229, 255, 235) : QColor(26, 29, 33, 200));
        p->drawRoundedRect(box, 3, 3);
        if (hot) {
            p->setBrush(Qt::NoBrush);
            p->setPen(QPen(QColor(255, 255, 255, 220), 1));
            p->drawRoundedRect(box, 3, 3);
        }
        p->setPen(hot ? QColor(26, 29, 33) : QColor(255, 255, 255, 240));
        p->drawText(box, Qt::AlignCenter, label);
    }
}

// 탑뷰 로봇 표시 — **노면표시 중장비**의 위에서 본 모습.
//
// 디자인 기준(실제 도로장비 도장 관습):
//   · 면취(chamfer)한 강판 섀시 — 둥근 모서리보다 '기계'로 읽힌다
//   · 세로 프레임 레일 2줄 — 구조물이 있다는 신호
//   · 데크 위 도료 드럼 + 컨트롤 박스 — 이 기계가 무슨 일을 하는지
//   · 전면 범퍼 **위험 사선**(호박/검정) — 중장비의 시각적 서명
//   · 후방 회전 경고등 — 강조색은 여기와 범퍼뿐
//
// ⚠️ 크기는 줄이지 않는다. 265mm 기체는 900×600 도면에서 실제로 1/4 을 차지하고,
//    그게 배치 판단의 근거다. 도면을 가리는 게 문제면 **로봇표시 토글로 끈다**
//    (Backend::setRobotVisible). 축척을 속이는 쪽으로 풀지 말 것.
// ⚠️ 화살표를 따로 그리지 않는다 — 면취한 앞코와 범퍼가 곧 진행 방향이다.
//
// 디테일은 화면상 크기(L)로 단계를 둔다. 작을 때 다 그리면 뭉개져서 노이즈만 된다.
void VideoView::paintRobot(QPainter *p, double sx, double sy, double ox, double oy)
{
    if (!m_robotValid) { paintPenMarker(p, sx, sy, ox, oy); return; }
    const QPointF w(m_robotPx.x() * sx + ox, m_robotPx.y() * sy + oy);

    // 기체 실측(2026-07-30 로봇팀 제원):
    //   전장 265mm · 휠베이스 166mm · 바퀴 지름 66mm · 높이 140mm
    //   바퀴 6개 = 가운데 고무 구동륜 2개(2륜 차동구동) + 옴니휠 4개
    // ⚠️ **폭은 제원에 없다.** 아래 kBodyWidM 은 전장 대비 비율 추정치다.
    //    실측이 나오면 이 상수만 바꾸면 된다.
    constexpr double kBodyLenM  = 0.265;   // 전장
    constexpr double kBodyWidM  = 0.212;   // ← 미실측(추정)
    constexpr double kWheelBase = 0.166;   // 앞뒤 옴니휠 축간
    constexpr double kWheelDia  = 0.066;   // 바퀴 지름

    // 실제 축척으로 그린다 — 900×600 도면에서 로봇이 얼마나 큰지가 그대로 보여야
    // 배치 판단이 된다.
    //
    // ⚠️ 상한을 140px 로 두고 있었는데, 그러면 **기본 배율에서 이미 잘린다**
    //    (900mm 도면이 620px 이면 265mm 기체 = 182px). 축척이 정확하다고 주석에
    //    써 놓고 실제로는 23% 작게 그리고 있었다. 게다가 노즐 점은 월드 좌표라
    //    배율대로 멀어지는데 몸통만 고정되니, 확대할수록 **노즐이 몸통에서 떨어져
    //    날아갔다**. 상한은 극단적 확대만 막는 값으로 올린다.
    const double L = qBound(16.0, kBodyLenM * m_tvPxPerM * sx, 420.0);
    const double W = L * (kBodyWidM / kBodyLenM);
    const double pxPerM = L / kBodyLenM;   // 아이콘 안에서 쓰는 축척

    // 색: 강판 회색 계열 + 호박색 강조 두 곳(범퍼·경고등)만. 몸통을 밝게 칠하면
    // 도면보다 로봇이 먼저 보인다 — 중장비는 원래 어둡고 경고색만 튄다.
    const QColor cBody(52, 57, 64);       // 섀시 강판
    const QColor cDeck(72, 79, 88);       // 데크 (한 톤 밝게 → 저상형 입체감)
    const QColor cRail(31, 34, 39);       // 프레임 레일
    const QColor cEdge(122, 131, 142);    // 판금 엣지 하이라이트
    const QColor cHazard(240, 176, 44);   // 위험 사선 · 경고등
    const QColor cWheel(20, 22, 26);      // 고무 구동륜 (2륜 차동구동)
    const QColor cOmni(78, 85, 95);       // 옴니휠 — 종동륜이라 한 톤 흐리게

    p->save();
    p->translate(w);
    p->setRenderHint(QPainter::Antialiasing, true);

    p->rotate(-m_robotThetaDeg);

    // 접지 그림자 — 도면 위에 떠 있는 물체로 읽히게 (평면 아이콘의 관습).
    // ⚠️ 반드시 rotate **뒤에** 그린다. 앞에서 그리면 로봇이 45° 로 섰을 때
    //    그림자만 축에 붙어 있어서 몸통 밖으로 삐져나온다.
    p->setPen(Qt::NoPen);
    p->setBrush(QColor(0, 0, 0, 52));
    p->drawRoundedRect(QRectF(-L / 2 + L * 0.02, -W / 2 + L * 0.03, L, W),
                       L * 0.06, L * 0.06);

    // ① 바퀴 6개 — 섀시 밖으로 살짝 나오게. 이게 있어야 '차량'으로 읽힌다.
    //    구동륜(가운데 고무 2개)만 진하게, 옴니휠 4개는 한 톤 흐리게 → 어느 쪽이
    //    움직임을 만드는지가 아이콘만 봐도 읽힌다.
    if (L >= 30.0) {
        const double wl = kWheelDia * pxPerM;          // 옆에서 본 바퀴 길이 = 지름
        const double wt = W * 0.15;                    // 두께(림 폭은 미실측)
        const double ax = (kWheelBase * 0.5) * pxPerM; // 앞뒤 옴니휠 축 위치
        p->setPen(Qt::NoPen);
        auto wheelAt = [&](double x, double y, const QColor &c) {
            p->setBrush(c);
            p->drawRoundedRect(QRectF(x - wl / 2, y - wt / 2, wl, wt),
                               wt * 0.45, wt * 0.45);
        };
        for (int j = -1; j <= 1; j += 2) {
            wheelAt(-ax, j * (W * 0.46), cOmni);        // 뒤 옴니휠
            wheelAt(+ax, j * (W * 0.46), cOmni);        // 앞 옴니휠
            wheelAt(0.0, j * (W * 0.50), cWheel);       // 가운데 고무 구동륜
        }
    }

    // ② 섀시 — 앞모서리를 크게, 뒷모서리를 작게 깎은 팔각 강판.
    //    둥근 사각형은 가전제품처럼 보인다. 면취한 직선 실루엣이 '기계'로 읽히고,
    //    앞뒤 깎임이 달라서 **방향 표시 없이도 어디가 앞인지** 알 수 있다.
    const double hw = W * 0.42;            // 섀시 반폭
    const double cf = L * 0.15;            // 앞 면취
    const double cr = L * 0.07;            // 뒤 면취
    QPainterPath body;
    body.moveTo(-L / 2 + cr, -hw);
    body.lineTo(L / 2 - cf, -hw);
    body.lineTo(L / 2,      -hw + cf);
    body.lineTo(L / 2,       hw - cf);
    body.lineTo(L / 2 - cf,  hw);
    body.lineTo(-L / 2 + cr, hw);
    body.lineTo(-L / 2,      hw - cr);
    body.lineTo(-L / 2,     -hw + cr);
    body.closeSubpath();
    p->setBrush(cBody);
    p->setPen(QPen(cEdge, std::max(1.0, L * 0.016)));
    p->drawPath(body);

    // ③ 세로 프레임 레일 2줄 — 강판 아래 구조물. 이것만으로 두께감이 생긴다.
    if (L >= 34.0) {
        p->setPen(Qt::NoPen);
        p->setBrush(cRail);
        for (int j = -1; j <= 1; j += 2)
            p->drawRect(QRectF(-L * 0.44, j * (hw * 0.62) - hw * 0.09,
                               L * 0.80, hw * 0.18));
    }

    // ④ 데크 플레이트 — 안쪽으로 물린 밝은 면. 저상형 기체의 평평한 윗면.
    if (L >= 26.0) {
        p->setBrush(cDeck);
        p->setPen(Qt::NoPen);
        p->drawRoundedRect(QRectF(-L * 0.38, -hw * 0.70, L * 0.70, hw * 1.40),
                           L * 0.03, L * 0.03);
    }

    // ⑤ 데크 적재물 — 도료 드럼(뒤) + 컨트롤 박스(앞).
    //    "이 기계가 무슨 일을 하는가" 를 말하는 유일한 부분이다.
    if (L >= 46.0) {
        const double dr = hw * 0.52;                       // 도료 드럼 반지름
        p->setBrush(QColor(38, 42, 48));
        p->setPen(QPen(cEdge, std::max(0.8, L * 0.010)));
        p->drawEllipse(QPointF(-L * 0.16, 0.0), dr, dr);   // 드럼 외곽
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(96, 105, 116));
        p->drawEllipse(QPointF(-L * 0.16, 0.0), dr * 0.34, dr * 0.34);  // 주입구

        p->setBrush(QColor(40, 44, 50));                   // 컨트롤 박스
        p->setPen(QPen(cEdge, std::max(0.8, L * 0.010)));
        p->drawRoundedRect(QRectF(L * 0.06, -hw * 0.42, L * 0.20, hw * 0.84),
                           L * 0.02, L * 0.02);
    }

    // ⑥ 전면 범퍼 — **위험 사선**(호박/검정). 중장비의 시각적 서명이자,
    //    화면에서 앞쪽을 단번에 찍어주는 표식. 강조색은 여기와 경고등뿐이다.
    const QRectF bump(L * 0.30, -hw * 0.96, L * 0.16, hw * 1.92);
    p->save();
    QPainterPath bumpPath;
    bumpPath.addRoundedRect(bump, L * 0.025, L * 0.025);
    p->setClipPath(bumpPath);
    p->fillRect(bump, QColor(26, 28, 32));
    if (L >= 32.0) {
        p->setPen(QPen(cHazard, std::max(1.6, L * 0.052)));
        for (double t = -hw * 2.2; t < hw * 2.2; t += std::max(4.0, L * 0.115))
            p->drawLine(QPointF(bump.left() - 2.0, t),
                        QPointF(bump.right() + 2.0, t - bump.width() * 2.0));
    } else {
        p->fillRect(bump, cHazard);        // 작을 때는 사선 대신 통칠
    }
    p->restore();
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(cEdge, std::max(0.8, L * 0.012)));
    p->drawPath(bumpPath);

    // ⚠️ 여기 있던 후방 회전 경고등(호박색 점)은 지웠다. 노즐 연결선이 붙는 자리와
    //    정확히 겹쳐서, 호박색 점이 **노즐로 오독**됐다. 실제 노즐은 흰 원이고 훨씬
    //    뒤에 있다. 강조색은 전면 범퍼 하나로 충분하다.

    // ⑧ 기준점 십자 = **ArUco 마커(ID 49) 중심** = 서버가 준 POSE(x, y) 그 자체.
    //    노즐은 여기가 아니라 155mm 뒤다 (Backend::pushPoseToView 참고).
    //    점 하나보다 십자가 낫다: 도료 드럼 위에서도 축이 어디인지 정확히 짚힌다.
    if (L >= 30.0) {
        const double ct = std::max(2.6, L * 0.075);
        p->setBrush(Qt::NoBrush);
        p->setPen(QPen(QColor(236, 242, 248, 200), std::max(1.0, L * 0.014)));
        p->drawLine(QPointF(-ct, 0), QPointF(ct, 0));
        p->drawLine(QPointF(0, -ct), QPointF(0, ct));
    }
    p->setPen(Qt::NoPen);
    p->setBrush(QColor(236, 242, 248, 235));
    p->drawEllipse(QPointF(0, 0), std::max(1.6, L * 0.030), std::max(1.6, L * 0.030));

    p->restore();

    paintPenMarker(p, sx, sy, ox, oy);
}

// 노즐(펜) 끝 — 페인트가 실제로 나오는 점. 기준점(ArUco 마커) 뒤 155mm 라 로봇 몸통에
// 가려진다 → **아이콘 위에** 찍는다. 전장 265mm 의 절반(132mm)보다 멀어서 섀시 밖으로
// 조금 나온다. 꼭짓점에서 후진·제자리회전할 때 어디를 칠하는지 이것만 보면 된다.
// 내려간 상태 = 채운 점(칠하는 중), 올라간 상태 = 빈 원(이동/회전).
void VideoView::paintPenMarker(QPainter *p, double sx, double sy, double ox, double oy)
{
    if (!m_penValid) return;
    const QPointF pw(m_penPx.x() * sx + ox, m_penPx.y() * sy + oy);
    const double r = qBound(5.0, 0.025 * m_tvPxPerM * sx, 11.0);

    // 기준점 → 노즐 연결선. 실측 오프셋 155mm (ArUco 마커 중심 ↔ 노즐).
    // 선이 없으면 꼭짓점에서 뒤로 물러났다 돌아오는 게 눈에 안 들어온다.
    if (m_robotValid) {
        const QPointF cw(m_robotPx.x() * sx + ox, m_robotPx.y() * sy + oy);
        p->setPen(QPen(QColor(20, 22, 26, 190), 3, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(cw, pw);
        p->setPen(QPen(QColor(255, 255, 255, 230), 1.4, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(cw, pw);
    }

    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(QColor(20, 22, 26, 210), r * 0.55));   // 어두운 테두리로 대비 확보
    p->drawEllipse(pw, r, r);
    p->setPen(QPen(QColor(255, 255, 255), r * 0.34));
    p->drawEllipse(pw, r, r);
    if (m_penDown) {
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(255, 255, 255));
        p->drawEllipse(pw, r * 0.52, r * 0.52);
    }
}

// ── 미션 설정 ─────────────────────────────────────────────────────────
void VideoView::setMissionPathsMeters(const QList<QList<QPointF>> &metersPaths,
                                      const QList<bool> &closed)
{
    m_missionPaths.clear();
    for (int k = 0; k < metersPaths.size(); ++k) {
        VVPath mp;
        mp.closed = closed.value(k, false);
        for (const QPointF &m : metersPaths[k])
            mp.pts.append(worldMmToTopPx(m.x() * 1000.0, m.y() * 1000.0));
        m_missionPaths.append(mp);
    }
    m_missionProgress = 0.0;
    update();
}

void VideoView::setMissionPathsPixels(const QList<QList<QPointF>> &imagePx,
                                      const QList<bool> &closed)
{
    m_missionPaths.clear();
    for (int k = 0; k < imagePx.size(); ++k) {
        VVPath mp;
        mp.closed = closed.value(k, false);
        mp.pts = QVector<QPointF>(imagePx[k].begin(), imagePx[k].end());
        m_missionPaths.append(mp);
    }
    m_missionProgress = 0.0;
    update();
}

void VideoView::setMissionProgress(double progress01)
{
    m_missionProgress = qBound(0.0, progress01, 1.0);
    update();
}

void VideoView::setRobotPose(double xM, double yM, double thetaDeg, bool valid)
{
    m_robotValid = valid;
    m_robotPx = worldMmToTopPx(xM * 1000.0, yM * 1000.0);
    m_robotThetaDeg = thetaDeg;
    update();
}

void VideoView::setPenMarker(double xM, double yM, bool down, bool valid)
{
    m_penValid = valid;
    m_penDown = down;
    m_penPx = worldMmToTopPx(xM * 1000.0, yM * 1000.0);
    update();
}

void VideoView::clearMission()
{
    m_missionPaths.clear();
    m_missionProgress = 0.0;
    m_robotValid = false;
    m_penValid = false;
    update();
}

// ── 다중 경로 관리 ────────────────────────────────────────────────────
void VideoView::stashActive()
{
    if (m_points.size() >= 2)
        m_done.append({ m_points, m_closed });
    m_points.clear();
    m_closed = false;
    resetSelection();
}

bool VideoView::activateDoneAt(const QPointF &img, double thr)
{
    for (int i = m_done.size() - 1; i >= 0; --i) {
        const VVPath &dp = m_done[i];
        bool hit = false;
        for (const QPointF &q : dp.pts) {
            if (QLineF(img, q).length() <= thr) { hit = true; break; }
        }
        if (!hit && dp.pts.size() >= 2) {
            // 선 위(도장 폭 안)를 클릭해도 잡히게
            const double band = std::max(thr, strokePx() * 0.5);
            for (int k = 1; k < dp.pts.size() && !hit; ++k)
                if (QLineF(dp.pts[k - 1], dp.pts[k]).length() > 1e-6) {
                    const QLineF seg(dp.pts[k - 1], dp.pts[k]);
                    const QPointF ab = seg.p2() - seg.p1();
                    const double len2 = ab.x() * ab.x() + ab.y() * ab.y();
                    double t = ((img.x() - seg.x1()) * ab.x() + (img.y() - seg.y1()) * ab.y()) / len2;
                    t = qBound(0.0, t, 1.0);
                    if (QLineF(img, seg.p1() + ab * t).length() <= band) hit = true;
                }
        }
        if (!hit && dp.closed && dp.pts.size() >= 3) {
            QPolygonF poly;
            for (const QPointF &q : dp.pts) poly << q;
            hit = poly.containsPoint(img, Qt::OddEvenFill);
        }
        if (!hit) continue;

        VVPath cur{ m_points, m_closed };
        m_points = m_done[i].pts;
        m_closed = m_done[i].closed;
        m_done.removeAt(i);
        if (cur.pts.size() >= 2)
            m_done.append(cur);
        // 도형 순서가 바뀌었다 — m_doneSel 인덱스가 딴 도형을 가리키게 되므로
        // 양쪽 선택을 같이 버린다.
        resetSelection();
        emit selectionChanged();
        emitPath();
        update();
        return true;
    }
    return false;
}

int VideoView::totalPointCount() const
{
    int n = m_points.size();
    for (const VVPath &dp : m_done) n += dp.pts.size();
    return n;
}

int VideoView::shapeCount() const
{
    return m_done.size() + (m_points.size() >= 2 ? 1 : 0);
}

// ── 선택 (활성 경로 + 완성 도형을 함께) ──────────────────────────────
void VideoView::syncDoneSelSize()
{
    while (m_doneSel.size() < m_done.size()) m_doneSel.append(QSet<int>());
    while (m_doneSel.size() > m_done.size()) m_doneSel.removeLast();
}

int VideoView::selectedPointCount() const
{
    int n = int(m_selection.size());
    for (const QSet<int> &s : m_doneSel) n += int(s.size());
    return n;
}

void VideoView::clearDoneSelection()
{
    for (QSet<int> &s : m_doneSel) s.clear();
}

// 선택 해제는 **반드시 여기를 지난다**.
// 예전에는 곳곳에서 m_selection.clear() 만 불렀는데, 글자처럼 획이 여러 개인
// 도면은 대부분의 점이 완성 도형(m_done) 쪽에 있어서 m_doneSel 이 그대로 남았다.
// → 빈 곳을 눌러도 파란 점이 안 꺼지고, 다음 드래그가 "안 보이는 선택"까지 끌고 갔다.
bool VideoView::resetSelection()
{
    syncDoneSelSize();
    if (selectedPointCount() == 0) return false;
    m_selection.clear();
    clearDoneSelection();
    emit selectionChanged();
    return true;
}

// 선택 전체를 끌어서 옮기기 시작한다.
// 핸들(크기 조절)은 applyToSelection 으로 완성 도형까지 건드리는데, 이동만
// m_points 밖에 안 움직여서 "늘리는 건 되는데 옮기는 건 안 된다"가 됐었다.
void VideoView::beginSelectionMove(const QPointF &img, bool onVertex)
{
    syncDoneSelSize();
    m_movingSel = true;
    m_selMoveDragged = false;
    m_selMoveOnVertex = onVertex;
    m_selMoveStartImg = img;
    m_selMoveStartPts = m_points;
    m_selMoveStartDone.clear();
    for (const VVPath &d : std::as_const(m_done)) m_selMoveStartDone.append(d.pts);
    beginPendingUndo();
}

// 선택된 점이 하나라도 있으면 그것만, 없으면 활성 도형 전체에 변환을 적용한다.
void VideoView::applyToSelection(const std::function<QPointF(const QPointF &)> &fn)
{
    syncDoneSelSize();
    if (selectedPointCount() > 0) {
        for (int i : std::as_const(m_selection))
            if (i >= 0 && i < m_points.size()) m_points[i] = fn(m_points[i]);
        for (int s = 0; s < m_done.size() && s < m_doneSel.size(); ++s)
            for (int i : std::as_const(m_doneSel[s]))
                if (i >= 0 && i < m_done[s].pts.size())
                    m_done[s].pts[i] = fn(m_done[s].pts[i]);
    } else {
        for (QPointF &p : m_points) p = fn(p);
    }
}

double VideoView::selectedArcRadiusPx(const QList<VVPath> &snapshot) const
{
    if (snapshot.isEmpty()) return 0.0;
    double minimum = std::numeric_limits<double>::infinity();
    auto consider = [&](const VVPath &path, bool fullySelected) {
        if (!fullySelected) return;
        const DisplayArc arc = displayArcFor(path.pts, path.closed);
        if (arc.ok) minimum = std::min(minimum, arc.radiusPx);
    };

    if (selectedPointCount() == 0) {
        consider(snapshot.first(), true);
    } else {
        consider(snapshot.first(), m_selection.size() == snapshot.first().pts.size());
        for (int i = 1; i < snapshot.size() && i - 1 < m_doneSel.size(); ++i)
            consider(snapshot[i], m_doneSel[i - 1].size() == snapshot[i].pts.size());
    }
    return std::isfinite(minimum) ? minimum : 0.0;
}

// 변환/핸들의 기준이 되는 사각형. 선택이 있으면 선택 점들, 없으면 활성 도형.
QRectF VideoView::selectionBoundsImg() const
{
    bool first = true;
    QRectF box;
    auto add = [&](const QPointF &p) {
        if (first) { box = QRectF(p, p); first = false; return; }
        box.setLeft(std::min(box.left(), p.x()));
        box.setRight(std::max(box.right(), p.x()));
        box.setTop(std::min(box.top(), p.y()));
        box.setBottom(std::max(box.bottom(), p.y()));
    };

    if (selectedPointCount() > 0) {
        for (int i : m_selection)
            if (i >= 0 && i < m_points.size()) add(m_points[i]);
        for (int s = 0; s < m_done.size() && s < m_doneSel.size(); ++s)
            for (int i : m_doneSel[s])
                if (i >= 0 && i < m_done[s].pts.size()) add(m_done[s].pts[i]);
    } else {
        for (const QPointF &p : m_points) add(p);
    }
    return first ? QRectF() : box;
}

// ── 되돌리기 ─────────────────────────────────────────────────────────
// 점 하나가 아니라 "방금 한 동작" 단위로 되돌린다.
// 모든 변경 함수는 바꾸기 직전에 이걸 부른다.
void VideoView::pushUndo()
{
    Snapshot s;
    s.done = m_done;
    s.pts = m_points;
    s.closed = m_closed;
    s.drawing = m_drawing;
    m_undo.append(s);
    if (m_undo.size() > 60) m_undo.removeFirst();
}

void VideoView::beginPendingUndo()
{
    m_pendingUndo.done = m_done;
    m_pendingUndo.pts = m_points;
    m_pendingUndo.closed = m_closed;
    m_pendingUndo.drawing = m_drawing;
    m_pendingUndoValid = true;
}

void VideoView::commitPendingUndo()
{
    if (!m_pendingUndoValid) return;
    m_pendingUndoValid = false;
    m_undo.append(m_pendingUndo);
    if (m_undo.size() > 60) m_undo.removeFirst();
}

// ── 변환 ─────────────────────────────────────────────────────────────
QList<int> VideoView::transformTargets() const
{
    QList<int> t;
    if (!m_selection.isEmpty()) {
        for (int i : m_selection)
            if (i >= 0 && i < m_points.size()) t << i;
    } else {
        for (int i = 0; i < m_points.size(); ++i) t << i;
    }
    return t;
}

// ── 선택 박스 + 크기조절/회전 핸들 ───────────────────────────────────
// 박스는 꼭짓점에서 살짝 떨어뜨려 그린다. 사각형 도형이면 박스 모서리와
// 도형 꼭짓점이 정확히 겹쳐서 "점 하나 옮기기"가 막히기 때문이다.
QRectF VideoView::selectionBoxImg() const
{
    if (m_frame.isNull()) return QRectF();
    // 편집 표시가 꺼져 있으면 박스도 핸들도 없다.
    // (핸들 히트테스트도 이 함수를 쓰므로 그림과 조작이 항상 같이 사라진다)
    if (!m_focused) return QRectF();
    QRectF box = selectionBoundsImg();     // 선택이 여러 도형에 걸쳐 있어도 전부 포함
    if (box.isNull()) return QRectF();
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    const double pad = (sx > 0) ? 13.0 / sx : 13.0;
    box.adjust(-pad, -pad, pad, pad);
    return box;
}

// 8개 핸들 + 회전 손잡이의 뷰 좌표 (그리기·히트테스트 공용)
static void handlePointsFor(const QRectF &r, QPointF out[9])
{
    out[0] = r.topLeft();      out[1] = r.topRight();
    out[2] = r.bottomRight();  out[3] = r.bottomLeft();
    out[4] = QPointF(r.center().x(), r.top());
    out[5] = QPointF(r.right(), r.center().y());
    out[6] = QPointF(r.center().x(), r.bottom());
    out[7] = QPointF(r.left(), r.center().y());
    out[8] = QPointF(r.center().x(), r.top() - 24.0);   // 회전
}

void VideoView::paintSelectionBox(QPainter *p, double sx, double sy, double ox, double oy)
{
    m_handleRects.clear();
    if (!m_isTopView || m_drawing || !m_interactive) return;
    const QRectF b = selectionBoxImg();
    if (b.isEmpty()) return;

    const QRectF r(b.x() * sx + ox, b.y() * sy + oy, b.width() * sx, b.height() * sy);
    if (r.width() < 16 || r.height() < 16) return;

    const bool partial = (selectedPointCount() > 0);
    p->setBrush(Qt::NoBrush);
    QPen boxPen(kSel, 1.0, Qt::DashLine);
    boxPen.setDashPattern({3, 3});
    p->setPen(boxPen);
    p->drawRect(r);

    QPointF hp[9];
    handlePointsFor(r, hp);
    m_handleRects.resize(9);

    // 회전 손잡이까지 잇는 짧은 선
    p->setPen(QPen(kSel, 1.0));
    p->drawLine(QPointF(r.center().x(), r.top()), hp[8]);

    for (int i = 0; i < 9; ++i) {
        const double hr = (i == 8) ? 5.0 : (i < 4 ? 4.5 : 3.8);
        m_handleRects[i] = QRectF(hp[i].x() - hr - 3, hp[i].y() - hr - 3,
                                  (hr + 3) * 2, (hr + 3) * 2);
        p->setPen(QPen(QColor(26, 29, 33, 220), 1.2));
        p->setBrush(partial ? QColor(Qt::white) : kSel);
        if (i == 8) p->drawEllipse(hp[i], hr, hr);
        else        p->drawRect(QRectF(hp[i].x() - hr, hp[i].y() - hr, hr * 2, hr * 2));
    }

    // 조작 중에는 도형을 가리지 않고, 손을 뗐을 때만 작은 안내처럼 보인다.
    if (m_handleIdx < 0) {
        p->setFont(QFont("Pretendard", 8));
        const QString tag = m_radiusConstraintHit
            ? QStringLiteral("최소 도색 R 200 mm · 더 줄일 수 없음")
            : (partial
               ? QStringLiteral("선택 %1점 · 모서리=비율 유지 · Shift=자유 변형")
                     .arg(selectedPointCount())
               : QStringLiteral("모서리=비율 유지 · Shift=자유 변형(ARC 해제) · 변=한쪽 조절"));
        const QFontMetrics fm(p->font());
        QRectF tb(r.left(), r.bottom() + 6, fm.horizontalAdvance(tag) + 10,
                  fm.height() + 4);
        if (tb.bottom() > height() - 2) tb.moveTop(r.top() - tb.height() - 6);
        tb.moveLeft(qBound(2.0, tb.left(),
                           std::max(2.0, width() - tb.width() - 2.0)));
        p->setPen(Qt::NoPen);
        p->setBrush(m_radiusConstraintHit ? QColor(92, 52, 18, 220)
                                          : QColor(26, 29, 33, 145));
        p->drawRoundedRect(tb, 3, 3);
        p->setPen(m_radiusConstraintHit ? QColor(255, 210, 135)
                                        : QColor(kSel.red(), kSel.green(), kSel.blue(), 185));
        p->drawText(tb, Qt::AlignCenter, tag);
    }
}

int VideoView::handleAtView(const QPointF &v) const
{
    for (int i = 0; i < m_handleRects.size(); ++i)
        if (m_handleRects[i].contains(v)) return i;
    return -1;
}

void VideoView::applyHandleDrag(const QPointF &img)
{
    if (m_handleIdx < 0 || m_boxImg.isEmpty()) return;
    // 드래그 시작 시점의 좌표를 원본으로 삼아야 누적 오차가 안 생긴다
    if (m_handleStartAll.isEmpty()) return;

    m_radiusConstraintHit = false;
    // 스냅샷을 되돌려놓고 변환을 새로 얹는다 (드래그 중 계속 호출되므로)
    m_points = m_handleStartAll.first().pts;
    m_closed = m_handleStartAll.first().closed;
    for (int s = 0; s + 1 < m_handleStartAll.size() && s < m_done.size(); ++s)
        m_done[s].pts = m_handleStartAll[s + 1].pts;

    // 회전 손잡이 — Shift 로 15° 스냅
    if (m_handleIdx == 8) {
        const QPointF c = m_boxImg.center();
        const double a0 = std::atan2(m_rotStartImg.y() - c.y(), m_rotStartImg.x() - c.x());
        const double a1 = std::atan2(img.y() - c.y(), img.x() - c.x());
        double d = a1 - a0;
        if (QGuiApplication::keyboardModifiers() & Qt::ShiftModifier) {
            const double step = 15.0 * M_PI / 180.0;
            d = std::round(d / step) * step;
        }
        const double cs = std::cos(d), sn = std::sin(d);
        applyToSelection([&](const QPointF &p) {
            const QPointF q = p - c;
            return QPointF(c.x() + q.x() * cs - q.y() * sn,
                           c.y() + q.x() * sn + q.y() * cs);
        });
        emitPath();
        update();
        return;
    }

    const bool corner = (m_handleIdx <= 3);
    const bool useX = corner || m_handleIdx == 5 || m_handleIdx == 7;
    const bool useY = corner || m_handleIdx == 4 || m_handleIdx == 6;
    const QPointF a = m_handleAnchor;
    // 화면에 표시되는 선택 박스는 조작하기 쉽도록 여백이 있다. 마우스 이동량을
    // 실제 도형 핸들에 적용해야 반대편 도형 경계가 움직이지 않는다.
    const QPointF handleImg = m_handleOrigin + (img - m_rotStartImg);
    double fx = 1.0, fy = 1.0;
    const double dx0 = m_handleOrigin.x() - a.x();
    const double dy0 = m_handleOrigin.y() - a.y();
    if (useX && std::abs(dx0) > 1e-6) fx = (handleImg.x() - a.x()) / dx0;
    if (useY && std::abs(dy0) > 1e-6) fy = (handleImg.y() - a.y()) / dy0;

    // 모서리는 기본적으로 비율을 유지한다. Shift를 누른 동안만 자유 변형한다.
    // 원을 평범하게 키웠다가 타원이 되어 ARC 피팅이 깨지는 일을 막기 위한 기본값이다.
    if (corner && !(QGuiApplication::keyboardModifiers() & Qt::ShiftModifier)) {
        const double f = (std::abs(fx) + std::abs(fy)) / 2.0;
        fx = (fx < 0 ? -f : f);
        fy = (fy < 0 ? -f : f);

        const double radiusPx = selectedArcRadiusPx(m_handleStartAll);
        if (radiusPx > 0.0 && m_tvPxPerM > 1e-9) {
            const double requested = std::min(std::abs(fx), std::abs(fy));
            const double constrained = motionprogram::constrainPaintArcScale(
                radiusPx / m_tvPxPerM, requested);
            if (constrained > requested + 1e-9) m_radiusConstraintHit = true;
            fx = std::copysign(constrained, fx);
            fy = std::copysign(constrained, fy);
        }
    }
    // 0 배로 찌그러뜨려 복구 불가가 되는 것만 막는다 (뒤집기는 허용)
    auto guard = [](double f) {
        if (std::abs(f) < 0.02) return f < 0 ? -0.02 : 0.02;
        return qBound(-40.0, f, 40.0);
    };
    fx = guard(fx);
    fy = guard(fy);

    applyToSelection([&](const QPointF &p) {
        return QPointF(a.x() + (p.x() - a.x()) * fx,
                       a.y() + (p.y() - a.y()) * fy);
    });
    emitPath();
    update();
}

// deg > 0 = 화면에서 반시계(좌회전) — 사용자가 보는 대로
void VideoView::rotateActive(double deg)
{
    const QRectF b = selectionBoundsImg();
    if (b.isNull()) return;
    pushUndo();
    const QPointF c = b.center();
    const double a = -deg * M_PI / 180.0;   // 이미지 y-down 보정
    const double ca = std::cos(a), sa = std::sin(a);
    applyToSelection([&](const QPointF &p) {
        const QPointF d = p - c;
        return QPointF(c.x() + d.x() * ca - d.y() * sa,
                       c.y() + d.x() * sa + d.y() * ca);
    });
    emitPath();
    update();
}

void VideoView::flipActive(bool horizontal)
{
    const QRectF b = selectionBoundsImg();
    if (b.isNull()) return;
    pushUndo();
    const QPointF c = b.center();
    applyToSelection([&](const QPointF &p) {
        return horizontal ? QPointF(2 * c.x() - p.x(), p.y())
                          : QPointF(p.x(), 2 * c.y() - p.y());
    });
    // 도형 전체를 뒤집으면 진행 방향도 뒤집히므로 순서까지 뒤집어 자연스럽게
    if (selectedPointCount() == 0)
        std::reverse(m_points.begin(), m_points.end());
    emitPath();
    update();
}

void VideoView::scaleActive(double factor)
{
    if (factor <= 0.01) return;
    const QRectF b = selectionBoundsImg();
    if (b.isNull()) return;
    QList<VVPath> snapshot;
    snapshot.append(VVPath{m_points, m_closed});
    for (const VVPath &path : std::as_const(m_done)) snapshot.append(path);
    const double radiusPx = selectedArcRadiusPx(snapshot);
    if (radiusPx > 0.0 && m_tvPxPerM > 1e-9) {
        const double constrained = motionprogram::constrainPaintArcScale(
            radiusPx / m_tvPxPerM, factor);
        m_radiusConstraintHit = constrained > factor + 1e-9;
        factor = constrained;
    } else {
        m_radiusConstraintHit = false;
    }
    pushUndo();
    const QPointF c = b.center();
    applyToSelection([&](const QPointF &p) { return c + (p - c) * factor; });
    emitPath();
    update();
}

// Ctrl+A — 활성 도형뿐 아니라 화면의 모든 도형을 선택한다
void VideoView::selectAllActive()
{
    syncDoneSelSize();
    m_focused = true;              // 선택했는데 박스가 안 보이면 안 된다
    m_selection.clear();
    for (int i = 0; i < m_points.size(); ++i) m_selection.insert(i);
    for (int s = 0; s < m_done.size(); ++s) {
        m_doneSel[s].clear();
        for (int i = 0; i < m_done[s].pts.size(); ++i) m_doneSel[s].insert(i);
    }
    emit selectionChanged();
    update();
}

// Delete — 선택된 점을 도형 구분 없이 모두 지운다.
// 어떤 도형의 점이 다 지워지면 그 도형 자체가 사라진다.
void VideoView::deleteSelection()
{
    syncDoneSelSize();
    if (selectedPointCount() == 0) return;
    pushUndo();

    if (!m_selection.isEmpty()) {
        QVector<QPointF> keep;
        for (int i = 0; i < m_points.size(); ++i)
            if (!m_selection.contains(i)) keep.append(m_points[i]);
        m_points = keep;
        if (m_points.size() < 3) m_closed = false;
        m_selection.clear();
    }

    for (int s = m_done.size() - 1; s >= 0; --s) {
        if (s >= m_doneSel.size() || m_doneSel[s].isEmpty()) continue;
        QVector<QPointF> keep;
        for (int i = 0; i < m_done[s].pts.size(); ++i)
            if (!m_doneSel[s].contains(i)) keep.append(m_done[s].pts[i]);
        if (keep.size() < 2) {
            m_done.removeAt(s);
            m_doneSel.removeAt(s);
        } else {
            m_done[s].pts = keep;
            if (keep.size() < 3) m_done[s].closed = false;
            m_doneSel[s].clear();
        }
    }

    // 활성 경로가 비었는데 완성 도형이 남아 있으면 하나를 활성으로 끌어올린다
    if (m_points.isEmpty() && !m_done.isEmpty()) {
        m_points = m_done.last().pts;
        m_closed = m_done.last().closed;
        m_done.removeLast();
        if (!m_doneSel.isEmpty()) m_doneSel.removeLast();
    }

    m_drawing = false;
    emit selectionChanged();
    emitPath();
    update();
}

// ── 점 병합 ───────────────────────────────────────────────────────────
double VideoView::mergeThresholdPx() const
{
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
    const double screen = (sx > 1e-6) ? kMergeScreenPx / sx : kMergeScreenPx;
    const double world  = (m_tvPxPerM > 1e-9) ? kMergeWorldMm / 1000.0 * m_tvPxPerM : 0.0;
    return std::max(screen, world);
}

int VideoView::mergeClosePoints()
{
    if (m_points.size() < 2) return 0;
    const double thr = mergeThresholdPx();
    QVector<QPointF> out;
    out.reserve(m_points.size());
    for (const QPointF &p : std::as_const(m_points)) {
        if (!out.isEmpty() && QLineF(out.last(), p).length() <= thr) continue;
        out.append(p);
    }
    while (m_closed && out.size() > 2 && QLineF(out.last(), out.first()).length() <= thr)
        out.removeLast();

    const int removed = m_points.size() - out.size();
    if (removed > 0 && out.size() >= 1) {
        m_points = out;
        if (m_points.size() < 3) m_closed = false;
        resetSelection();
        emitPath();
        update();
        emit pointsMerged(removed);
    }
    return removed;
}

// ── 상호작용 ──────────────────────────────────────────────────────────
void VideoView::commitDrawPoint(QPointF img)
{
    pushUndo();
    if (!m_drawing || m_closed) return;
    if ((QGuiApplication::keyboardModifiers() & Qt::ControlModifier) && !m_points.isEmpty()) {
        const QPointF last = m_points.last();
        if (std::abs(img.x() - last.x()) > std::abs(img.y() - last.y())) img.setY(last.y());
        else img.setX(last.x());
    }

    const double thr = mergeThresholdPx();
    double best = thr; int bi = -1;
    for (int i = 0; i < m_points.size(); ++i) {
        const double d = QLineF(img, m_points[i]).length();
        if (d <= best) { best = d; bi = i; }
    }
    if (bi >= 0) {
        if (bi == m_points.size() - 1) {
            emit pointsMerged(1);
            update();
            return;
        }
        if (bi == 0 && m_points.size() >= 3) {
            m_closed = true;
            emitPath();
            emit pathClosed();
            finishFreeDraw();
            return;
        }
        img = m_points[bi];
    }

    m_points.append(img);
    emitPath();
    update();
}

void VideoView::finishFreeDraw()
{
    if (m_clickTimer) m_clickTimer->stop();
    m_pendingClick = false;
    if (!m_drawing) return;
    m_drawing = false;
    mergeClosePoints();
    emitPath();
    emit freeDrawEnded();
    update();
}

void VideoView::finishDraw()
{
    finishFreeDraw();
}

void VideoView::wheelEvent(QWheelEvent *e)
{
    if (!m_isTopView || !m_interactive || m_frame.isNull()) { e->ignore(); return; }
    const int dy = e->angleDelta().y();
    if (dy == 0) { e->ignore(); return; }
    zoomAtView(e->position().x(), e->position().y(), dy > 0 ? 1.12 : 1.0 / 1.12);
    e->accept();
}

void VideoView::mousePressEvent(QMouseEvent *e)
{
    if (!m_isTopView || !m_interactive) { e->ignore(); return; }
    forceActiveFocus();

    // 화면 이동: 휠클릭 드래그 · Alt+좌드래그 (좌클릭 편집을 막지 않는 조합)
    if (e->button() == Qt::MiddleButton
        || (e->button() == Qt::LeftButton && (e->modifiers() & Qt::AltModifier))) {
        m_panning = true;
        m_panStartView = e->position();
        m_panStartX = m_panX;
        m_panStartY = m_panY;
        setCursor(QCursor(Qt::ClosedHandCursor));
        e->accept();
        return;
    }

    if (e->button() == Qt::RightButton) {
        if (m_drawing) { finishFreeDraw(); e->accept(); return; }
        e->ignore();
        return;
    }
    if (e->button() != Qt::LeftButton) return;

    // 영역 확대 모드: 좌드래그가 "이 사각형만큼 확대"
    if (m_zoomTool && !m_drawing) {
        m_rubber = true;
        m_rubberZoom = true;
        m_rubberStart = m_rubberEnd = e->position();
        update();
        e->accept();
        return;
    }

    const QPointF img = mapToImage(e->position());
    double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);

    if (m_drawing) {
        if (e->flags() & Qt::MouseEventCreatedDoubleClick) {
            finishFreeDraw();
            e->accept();
            return;
        }
        m_pendingImg = img;
        m_pendingClick = true;
        e->accept();
        return;
    }

    const bool additive = e->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier);
    const double thr = (sx > 0) ? 12.0 / sx : 12.0;

    m_movingShape = false; m_dragIdx = -1; m_rubber = false;
    m_handleIdx = -1; m_movingSel = false;

    // 0) 선택 박스 핸들 → 크기 조절 / 회전
    //    ⚠️ 배지(회전각·변 길이)보다 **먼저** 본다. 배지는 장식이고 핸들은 조작
    //    수단이라, 겹치면 조작 쪽이 이겨야 한다. 예전에는 배지를 먼저 봐서
    //    핸들 위에 배지가 얹히면 확대·회전 드래그가 통째로 안 먹었다.
    {
        const int h = handleAtView(e->position());
        if (h >= 0) {
            m_boxImg = selectionBoxImg();
            if (!m_boxImg.isEmpty()) {
                beginPendingUndo();
                m_handleIdx = h;
                // 드래그 시작 시점의 모든 도형 좌표를 보관 (0=활성, 1..=완성)
                m_handleStartAll.clear();
                VVPath act; act.pts = m_points; act.closed = m_closed;
                m_handleStartAll.append(act);
                for (const VVPath &d : std::as_const(m_done)) m_handleStartAll.append(d);
                m_rotStartImg = img;
                QPointF hp[9];
                // 표시/히트 테스트용 여백을 제외한 실제 도형 경계를 변환 기준으로 쓴다.
                const QRectF geometryBox = selectionBoundsImg();
                handlePointsFor(geometryBox, hp);
                m_handleOrigin = hp[h];
                // 반대편 모서리/변이 고정점
                static const int opposite[8] = { 2, 3, 0, 1, 6, 7, 4, 5 };
                m_handleAnchor = (h == 8) ? geometryBox.center() : hp[opposite[h]];
                update(); // 누르는 동안 반투명 조작 안내를 숨긴다.
                e->accept();
                return;
            }
        }
    }

    // 1) 활성 경로의 꼭짓점
    int hitIdx = -1;
    double best = thr;
    for (int i = 0; i < m_points.size(); ++i) {
        const double d = QLineF(img, m_points[i]).length();
        if (d <= best) { best = d; hitIdx = i; }
    }
    if (hitIdx >= 0) {
        m_focused = true;          // 점을 집었다 = 다시 편집 모드
        if (additive) {
            // Ctrl/Shift+클릭 = 하나씩 선택 토글
            if (m_selection.contains(hitIdx)) m_selection.remove(hitIdx);
            else m_selection.insert(hitIdx);
            emit selectionChanged();
            update();
            e->accept();
            return;
        }
        if (!m_selection.contains(hitIdx)) {
            // 선택 밖의 점을 새로 잡았다 — 완성 도형 쪽 선택도 같이 버린다.
            // (안 버리면 이 점 하나를 끄는데 딴 획들이 통째로 따라온다)
            resetSelection();
            m_selection.insert(hitIdx);
            emit selectionChanged();
        }
        // 여러 점이 잡혀 있으면 그 점 하나가 아니라 **선택 전체**를 옮긴다.
        if (selectedPointCount() > 1) {
            beginSelectionMove(img, true);
            e->accept();
            return;
        }
        m_dragIdx = hitIdx;
        m_dragStartImg = img;
        m_dragStartPts = m_points;
        beginPendingUndo();
        e->accept();
        return;
    }

    // 2) 배지 — 핸들·꼭짓점을 다 놓친 자리에서만 받는다
    {
        // 회전각 배지 클릭 → 각도 수치 입력
        const int tb = turnBadgeAtView(e->position().x(), e->position().y());
        if (tb >= 0) {
            const QRectF box = m_turnBadgeRects.value(tb);
            emit turnEditRequested(tb, turnAngleAt(tb), box.center().x(), box.center().y());
            e->accept();
            return;
        }
        // 변 길이 라벨 클릭 → 수치 입력
        const int edge = edgeAtView(e->position().x(), e->position().y());
        if (edge >= 0) {
            const QRectF box = m_edgeLabelRects.value(edge);
            emit edgeEditRequested(edge, edgeLengthMm(edge), box.center().x(), box.center().y());
            e->accept();
            return;
        }
    }

    // 3) 선택 박스 안쪽 → 선택 전체 이동
    //    핸들은 박스 **테두리**에만 있어서, 안쪽을 끌면 여기까지 안 오고 5)번
    //    올가미로 떨어졌다 — 선택이 통째로 풀리면서 "안 옮겨진다"가 됐다.
    //    Ctrl/Shift 를 누른 채면 이 분기를 건너뛰어 안쪽에서도 올가미를 쓸 수 있다.
    if (!additive && selectedPointCount() > 0) {
        const QRectF box = selectionBoxImg();
        if (!box.isEmpty() && box.contains(img)) {
            beginSelectionMove(img, false);
            e->accept();
            return;
        }
    }

    // 4) 활성 닫힌 도형 내부 → 도형 전체 이동
    if (m_closed && m_points.size() >= 3) {
        QPolygonF poly;
        for (const QPointF &q : std::as_const(m_points)) poly << q;
        if (poly.containsPoint(img, Qt::OddEvenFill)) {
            m_focused = true;      // 도형을 잡았다 = 다시 편집 모드
            m_movingShape = true;
            m_moveStartImg = img;
            m_moveStartPts = m_points;
            beginPendingUndo();
            e->accept();
            return;
        }
    }

    // 5) 완성 도형 클릭 → 활성으로 승격
    if (activateDoneAt(img, thr)) {
        m_focused = true;          // 도형을 골랐다 = 다시 편집 모드
        e->accept();
        return;
    }

    // 6) 빈 곳 → 선택 해제 + 편집 표시 끄기, 그리고 올가미 시작
    //    도면을 다 그린 뒤에도 선택 박스·치수 배지가 화면에 계속 박혀 있어
    //    결과를 볼 수 없던 문제. 빈 곳을 누르면 완성 도형과 똑같이 보이고,
    //    도형이나 점을 다시 클릭하면 편집 표시가 돌아온다.
    if (!additive) {
        resetSelection();
        m_focused = false;
    }
    m_rubber = true;
    m_rubberZoom = false;
    m_rubberStart = e->position();
    m_rubberEnd = e->position();
    update();
    e->accept();
}

void VideoView::mouseMoveEvent(QMouseEvent *e)
{
    m_mouse = e->position(); m_hover = true;

    if (m_panning) {
        m_panX = m_panStartX + (e->position().x() - m_panStartView.x());
        m_panY = m_panStartY + (e->position().y() - m_panStartView.y());
        clampPan();
        emit scaleChanged();
        update();
        return;
    }

    if (m_rubber) {
        m_rubberEnd = e->position();
        update();
        return;
    }

    if (m_handleIdx >= 0) {
        commitPendingUndo();
        applyHandleDrag(mapToImage(e->position()));
        return;
    }

    // 선택 전체 이동 — 활성 경로와 완성 도형을 같은 델타로 함께 민다.
    // 시작 좌표 스냅샷 기준이라 드래그가 길어져도 오차가 안 쌓인다.
    if (m_movingSel) {
        syncDoneSelSize();
        const QPointF cur = mapToImage(e->position());
        const QPointF d = cur - m_selMoveStartImg;
        // 손떨림은 이동으로 치지 않는다 — 클릭만 한 건지 끈 건지 가르는 기준이다.
        double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
        const double slop = (sx > 0) ? 3.0 / sx : 3.0;
        if (!m_selMoveDragged && QLineF(QPointF(), d).length() <= slop) {
            update();
            return;                     // 아직 클릭인지 드래그인지 모른다 — 건드리지 않는다
        }
        m_selMoveDragged = true;
        commitPendingUndo();
        for (int i : std::as_const(m_selection))
            if (i >= 0 && i < m_points.size() && i < m_selMoveStartPts.size())
                m_points[i] = m_selMoveStartPts[i] + d;
        for (int s = 0; s < m_done.size() && s < m_doneSel.size()
                        && s < m_selMoveStartDone.size(); ++s)
            for (int i : std::as_const(m_doneSel[s]))
                if (i >= 0 && i < m_done[s].pts.size() && i < m_selMoveStartDone[s].size())
                    m_done[s].pts[i] = m_selMoveStartDone[s][i] + d;
        emitPath();
        update();
        return;
    }

    if (m_movingShape && !m_moveStartPts.isEmpty()) {
        commitPendingUndo();
        const QPointF d = mapToImage(e->position()) - m_moveStartImg;
        for (int i = 0; i < m_points.size() && i < m_moveStartPts.size(); ++i)
            m_points[i] = m_moveStartPts[i] + d;
        emitPath();
    } else if (!m_drawing && m_dragIdx >= 0 && !m_dragStartPts.isEmpty()) {
        // 여기는 **점 하나**만 온다. 여러 점이 잡힌 경우는 mousePressEvent 에서
        // m_movingSel(선택 전체 이동)로 빠지므로, 예전의 "선택이 여럿이면 같이 민다"
        // 분기는 도달할 수 없어 지웠다 (그 분기는 완성 도형을 못 움직였다).
        commitPendingUndo();
        if (m_dragIdx < m_points.size())
            m_points[m_dragIdx] = mapToImage(e->position());
        emitPath();
    }
    update();
}

void VideoView::mouseDoubleClickEvent(QMouseEvent *e)
{
    if (!m_isTopView || !m_interactive) { e->ignore(); return; }
    if (e->button() != Qt::LeftButton) { e->ignore(); return; }

    if (m_clickTimer) m_clickTimer->stop();
    m_pendingClick = false;
    if (m_drawing) {
        finishFreeDraw();
        e->accept();
        return;
    }

    // 점을 더블클릭하면 그 자리에서 (x, y) mm 를 보여주고 고칠 수 있게 한다.
    const int vi = vertexAtView(e->position().x(), e->position().y());
    if (vi >= 0) {
        const QPointF w = vertexWorldMm(vi);
        emit vertexEditRequested(vi, w.x(), w.y(), e->position().x(), e->position().y());
        e->accept();
        return;
    }
    e->accept();
}

void VideoView::mouseReleaseEvent(QMouseEvent *e)
{
    if (m_panning) {
        m_panning = false;
        setCursor(QCursor(Qt::ArrowCursor));
        return;
    }

    if (m_handleIdx >= 0) {
        m_handleIdx = -1;
        m_handleStartAll.clear();
        m_pendingUndoValid = false;
        // 크기 조절은 점의 위치만 바꾸는 변환이다. 작게 줄였다는 이유로 인접점을
        // 영구 삭제하면 다시 키워도 원래 곡률을 복원할 수 없고 ARC가 조각난다.
        // 점 하나를 직접 끌어 겹친 경우의 병합은 아래 별도 경로에서 계속 수행한다.
        update();
        return;
    }

    if (m_rubber) {
        m_rubber = false;
        const QRectF r = QRectF(m_rubberStart, m_rubberEnd).normalized();
        if (m_rubberZoom) {
            m_rubberZoom = false;
            // 너무 작은 사각이면 그냥 클릭으로 보고 한 단계만 확대
            if (r.width() > 12 && r.height() > 12)
                zoomToViewRect(r.x(), r.y(), r.width(), r.height());
            else
                zoomAtView(r.center().x(), r.center().y(), 1.5);
            update();
            return;
        }
        if (r.width() > 3 && r.height() > 3) {
            // 올가미는 활성 경로뿐 아니라 완성 도형까지 함께 잡는다.
            // (활성 것만 잡으면 화면을 다 긁어도 두어 점만 선택되고,
            //  Delete 를 눌러도 일부만 지워진다)
            syncDoneSelSize();
            double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
            auto inRect = [&](const QPointF &ip) {
                return r.contains(QPointF(ip.x() * sx + ox, ip.y() * sy + oy));
            };
            for (int i = 0; i < m_points.size(); ++i)
                if (inRect(m_points[i])) m_selection.insert(i);
            for (int s = 0; s < m_done.size(); ++s)
                for (int i = 0; i < m_done[s].pts.size(); ++i)
                    if (inRect(m_done[s].pts[i])) m_doneSel[s].insert(i);
            // 🔴 올가미로 뭔가 잡았으면 편집 표시를 **반드시 다시 켠다**.
            //    올가미는 빈 곳에서 시작하므로 mousePressEvent 6번 분기가 방금
            //    m_focused=false 로 껐다. 여기서 안 켜면 선택은 됐는데 박스도
            //    핸들도 안 그려지고, "박스 안쪽 드래그로 이동"이 박스가 빈 것으로
            //    판정돼 통째로 죽는다.
            if (selectedPointCount() > 0) m_focused = true;
            emit selectionChanged();
        }
        update();
        return;
    }

    if (m_drawing && m_pendingClick && e && e->button() == Qt::LeftButton) {
        const int interval = QGuiApplication::styleHints()->mouseDoubleClickInterval();
        m_clickTimer->start(interval + 40);
    }
    // 박스 안쪽을 **끌지 않고 그냥 클릭**했다 = 빈 곳 클릭으로 본다 → 선택 해제.
    // 전체 선택 시 박스가 화면을 다 덮어 선택을 풀 방법이 사라지는 걸 막는다.
    // (선택된 점을 직접 집은 경우는 해제하지 않는다 — 잡았다 놨을 뿐이다)
    if (m_movingSel && !m_selMoveDragged && !m_selMoveOnVertex) {
        resetSelection();
        m_focused = false;         // 빈 곳 클릭과 결과가 같아야 한다 (박스도 같이 꺼짐)
        update();
    }

    // ⚠️ 선택 이동 뒤에는 병합하지 않는다 — 획 여러 개를 겹쳐 놓은 자리에서
    //    가까운 점끼리 붙어버리면 글자가 뭉개진다.
    if (m_dragIdx >= 0 && m_selection.size() <= 1 && !m_movingShape && !m_movingSel)
        mergeClosePoints();

    m_pendingUndoValid = false;   // 안 움직였으면 되돌리기에 올리지 않는다
    m_dragIdx = -1;
    m_dragStartPts.clear();
    m_movingShape = false;
    m_moveStartPts.clear();
    m_movingSel = false;
    m_selMoveStartPts.clear();
    m_selMoveStartDone.clear();
}

void VideoView::hoverMoveEvent(QHoverEvent *e)
{
    m_mouse = e->position(); m_hover = true;

    if (m_zoomTool && m_isTopView && m_interactive) {
        setCursor(QCursor(Qt::CrossCursor));
        return;
    }

    const int wasEdge = m_hoverEdge;
    const int wasVert = m_hoverVertex;
    const bool editable = m_isTopView && m_interactive && !m_drawing;

    const int badge = editable ? turnBadgeAtView(e->position().x(), e->position().y()) : -1;
    m_hoverEdge = (editable && badge < 0)
                      ? edgeAtView(e->position().x(), e->position().y())
                      : -1;

    m_hoverVertex = -1;
    if (editable && badge < 0 && m_hoverEdge < 0 && !m_frame.isNull()) {
        double sx, sy, ox, oy; displayTransform(sx, sy, ox, oy);
        const QPointF img = mapToImage(e->position());
        const double thr = (sx > 0) ? 12.0 / sx : 12.0;
        double best = thr;
        for (int i = 0; i < m_points.size(); ++i) {
            const double d = QLineF(img, m_points[i]).length();
            if (d <= best) { best = d; m_hoverVertex = i; }
        }
    }

    // 핸들 위에서는 방향에 맞는 크기조절 커서
    int hnd = -1;
    if (editable && badge < 0 && m_hoverEdge < 0 && m_hoverVertex < 0)
        hnd = handleAtView(e->position());

    Qt::CursorShape shape = Qt::ArrowCursor;
    if (badge >= 0 || m_hoverEdge >= 0)       shape = Qt::PointingHandCursor;
    else if (m_hoverVertex >= 0)              shape = Qt::SizeAllCursor;
    else if (hnd == 0 || hnd == 2)            shape = Qt::SizeFDiagCursor;
    else if (hnd == 1 || hnd == 3)            shape = Qt::SizeBDiagCursor;
    else if (hnd == 4 || hnd == 6)            shape = Qt::SizeVerCursor;
    else if (hnd == 5 || hnd == 7)            shape = Qt::SizeHorCursor;
    else if (hnd == 8)                        shape = Qt::CrossCursor;
    // 선택 박스 안쪽 = 통째로 이동. 커서로 알려주지 않으면 끌 수 있는 줄 모른다.
    else if (editable && selectedPointCount() > 0) {
        const QRectF box = selectionBoxImg();
        if (!box.isEmpty() && box.contains(mapToImage(e->position())))
            shape = Qt::SizeAllCursor;
    }
    setCursor(QCursor(shape));

    if (m_hoverEdge != wasEdge || m_hoverVertex != wasVert || badge >= 0 || hnd >= 0)
        update();
    if (m_drawing) update();
}

// ── 꼭짓점 회전각 (프로토콜 TURN: 양수 = 좌회전) ─────────────────────
int VideoView::turnBadgeAtView(qreal viewX, qreal viewY) const
{
    if (!m_isTopView || !m_showLabels || !m_focused || m_points.size() > kDenseLimit)
        return -1;
    const QPointF v(viewX, viewY);
    for (int i = 0; i < m_turnBadgeRects.size(); ++i) {
        const QRectF &r = m_turnBadgeRects[i];
        if (!r.isNull() && r.adjusted(-2, -2, 2, 2).contains(v))
            return i;
    }
    return -1;
}

double VideoView::turnAngleAt(int index) const
{
    const int n = m_points.size();
    if (n < 3 || index < 0 || index >= n) return 0.0;
    if (!m_closed && (index == 0 || index == n - 1)) return 0.0;
    const QPointF prev = m_points[(index - 1 + n) % n];
    const QPointF cur  = m_points[index];
    const QPointF next = m_points[(index + 1) % n];
    const QPointF v1 = cur - prev, v2 = next - cur;
    // 화면 y는 아래로 향하므로 월드 회전 부호는 외적을 반전한다
    const double crossW = -(v1.x() * v2.y() - v1.y() * v2.x());
    const double dot = v1.x() * v2.x() + v1.y() * v2.y();
    return std::atan2(crossW, dot) * 180.0 / M_PI;
}

// 이 꼭짓점의 회전각이 deg 가 되도록 뒤쪽 경로를 통째로 돌린다.
// (앞쪽은 이미 지나온 길이므로 건드리지 않는 것이 직관적이다)
bool VideoView::setTurnAngleAt(int index, double deg)
{
    const int n = m_points.size();
    if (n < 3 || index < 1 || index >= n - (m_closed ? 0 : 1)) return false;
    deg = qBound(-179.0, deg, 179.0);
    const double cur = turnAngleAt(index);
    double delta = deg - cur;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    if (std::abs(delta) < 1e-6) return true;
    pushUndo();

    const double a = -delta * M_PI / 180.0;   // 이미지 y-down 보정
    const double ca = std::cos(a), sa = std::sin(a);
    const QPointF pivot = m_points[index];
    for (int i = index + 1; i < n; ++i) {
        const QPointF d = m_points[i] - pivot;
        m_points[i] = QPointF(pivot.x() + d.x() * ca - d.y() * sa,
                              pivot.y() + d.x() * sa + d.y() * ca);
    }
    emitPath();
    update();
    return true;
}

// ── 변 길이(mm) 수치 편집 ─────────────────────────────────────────────
int VideoView::edgeAtView(qreal viewX, qreal viewY) const
{
    if (!m_isTopView || !m_showLabels || !m_focused || m_points.size() > kDenseLimit)
        return -1;
    const QPointF v(viewX, viewY);
    for (int i = 0; i < m_edgeLabelRects.size(); ++i) {
        const QRectF &r = m_edgeLabelRects[i];
        if (!r.isNull() && r.adjusted(-2, -2, 2, 2).contains(v))
            return i;
    }
    return -1;
}

double VideoView::edgeLengthMm(int index) const
{
    const int n = m_points.size();
    if (n < 2 || index < 0) return 0.0;
    const int nSeg = (m_closed && n > 2) ? n : n - 1;
    if (index >= nSeg) return 0.0;
    const double spm = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
    return QLineF(m_points[index], m_points[(index + 1) % n]).length() / spm * 1000.0;
}

bool VideoView::setEdgeLengthMm(int index, double mm)
{
    pushUndo();
    const int n = m_points.size();
    if (n < 2 || index < 0 || mm <= 0.05) return false;
    const int nSeg = (m_closed && n > 2) ? n : n - 1;
    if (index >= nSeg) return false;

    const int ia = index, ib = (index + 1) % n;
    const QPointF a = m_points[ia], b = m_points[ib];
    const double len = QLineF(a, b).length();
    if (len < 1e-6) return false;

    const double spm = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
    const double targetPx = mm / 1000.0 * spm;
    if (std::abs(targetPx - len) < 1e-6) return true;

    const QPointF u((b.x() - a.x()) / len, (b.y() - a.y()) / len);
    const QPointF delta = u * (targetPx - len);

    if (m_closed && n > 2) {
        const double pa = a.x() * u.x() + a.y() * u.y();
        const double pb = b.x() * u.x() + b.y() * u.y();
        const double mid = (pa + pb) * 0.5;
        for (int k = 0; k < n; ++k) {
            const double pk = m_points[k].x() * u.x() + m_points[k].y() * u.y();
            if (pk > mid) m_points[k] += delta;
        }
    } else {
        for (int k = ib; k < n; ++k)
            m_points[k] += delta;
    }

    emitPath();
    update();
    return true;
}

// ── 작도 조작 ─────────────────────────────────────────────────────────
void VideoView::startDraw()
{
    if (m_clickTimer) m_clickTimer->stop();
    m_pendingClick = false;
    stashActive();
    m_focused = true;
    m_drawing = true;
    emitPath();
    update();
}

void VideoView::clearPath()
{
    if (!m_points.isEmpty() || !m_done.isEmpty()) pushUndo();
    if (m_clickTimer) m_clickTimer->stop();
    m_pendingClick = false;
    m_points.clear();
    m_closed = false;
    m_done.clear();
    m_drawing = false;
    m_hoverEdge = -1;
    m_hoverVertex = -1;
    m_edgeLabelRects.clear();
    resetSelection();
    emitPath();
    update();
}

// Ctrl+Z — "방금 한 동작"을 통째로 되돌린다.
// 예전에는 점을 하나씩 지워서, 프리셋 하나를 놓고 되돌리면 꼭짓점이 한 개씩
// 사라졌다. 이제는 모든 변경 함수가 pushUndo() 로 직전 상태를 쌓아두고
// 여기서 그 상태를 그대로 복원한다.
// (자유 작도 중에는 클릭 한 번이 한 동작이라 점 하나씩 취소되는 게 맞다)
void VideoView::undo()
{
    if (m_undo.isEmpty()) return;
    const Snapshot s = m_undo.takeLast();
    m_done = s.done;
    m_points = s.pts;
    m_closed = s.closed;
    m_drawing = s.drawing;

    m_focused = true;              // 되돌린 결과를 바로 손볼 수 있게
    m_selection.clear();
    m_doneSel.clear();
    syncDoneSelSize();
    m_dragIdx = -1;
    m_handleIdx = -1;
    m_hoverEdge = -1;
    m_hoverVertex = -1;
    m_edgeLabelRects.clear();
    m_turnBadgeRects.clear();

    emit selectionChanged();
    emitPath();
    update();
}

int VideoView::undoDepth() const
{
    return int(m_undo.size());
}

void VideoView::setPresetShape(const QString &type)
{
    if (m_frame.isNull()) return;
    buildPreset(type, m_frame.width() / 2.0, m_frame.height() / 2.0);
}

void VideoView::addPresetAt(const QString &type, qreal viewX, qreal viewY)
{
    if (m_frame.isNull()) return;
    const QPointF c = mapToImage(QPointF(viewX, viewY));
    buildPreset(type, c.x(), c.y());
}

// 중심(cx,cy) 기준 프리셋 생성. 기존 도형은 유지된다.
void VideoView::buildPreset(const QString &type, double cx, double cy)
{
    double s = std::max(40.0, std::min(m_frame.width(), m_frame.height()) / 6.0);
    pushUndo();
    stashActive();
    m_focused = true;              // 방금 넣은 것은 바로 편집할 수 있어야 한다
    m_drawing = false;

    auto addDone = [this](std::initializer_list<QPointF> pts, bool closed) {
        VVPath dp;
        dp.closed = closed;
        for (const QPointF &q : pts) dp.pts.append(q);
        m_done.append(dp);
    };

    if (type == "RECT") {
        m_closed = true;
        m_points << QPointF(cx - s, cy - s) << QPointF(cx + s, cy - s)
                 << QPointF(cx + s, cy + s) << QPointF(cx - s, cy + s);
    } else if (type == "TRIANGLE") {
        m_closed = true;
        m_points << QPointF(cx, cy - s) << QPointF(cx + s, cy + s) << QPointF(cx - s, cy + s);
    } else if (type == "CIRCLE") {
        // ⚠️ 예전엔 6점(60° 간격) 육각형이었다. 화면에서도 각져 보였고, 무엇보다
        //    서버로 나갈 때 ARC 가 아니라 MOVE 6개가 됐다. 프로토콜 §ARC 는
        //    'D'·'O'·곡선 표지를 원호로 보내라고 못박고 있으므로 실제 원으로 만든다.
        //    10° 간격이면 반지름 대비 현 오차가 0.04% 라 눈으로도 원이다.
        //    점 개수는 병합 임계값에 맞춰 정한다. 촘촘하게 깔면 예쁘지만, 점 하나를
        //    끌자마자 mergeClosePoints() 가 이웃을 통째로 지워 원이 무너진다.
        m_closed = true;
        // 새 원을 만든 직후부터 서버가 거부하는 상태가 되면 안 된다. 캔버스의
        // 실제 축척을 기준으로 최소 도색 반지름에서 시작한다.
        s = std::max(s, motionprogram::kServerConfirmedMinPaintRadiusM * m_tvPxPerM);
        const double minStep = std::max(mergeThresholdPx() * 1.3, 6.0);
        const int nPt = qBound(16, int(std::floor(2.0 * M_PI * s / minStep)), 36);
        for (int i = 0; i < nPt; ++i) {
            const double a = i * (2.0 * M_PI / nPt);
            m_points << QPointF(cx + s * std::cos(a), cy + s * std::sin(a));
        }
    } else if (type == "LINE") {
        m_closed = false;
        m_points << QPointF(cx - s, cy) << QPointF(cx + s, cy);
    } else if (type == "CROSSWALK") {
        // 횡단보도: 줄마다 "한 줄 긋기" — 도장 폭(50mm)이 곧 줄 두께가 되도록
        // 닫힌 사각이 아니라 직선 세그먼트로 만든다.
        const double bh = 1.7 * s, gap = std::max(strokePx() * 1.6, 0.5 * s);
        const int n = 5;
        const double x0 = cx - gap * (n - 1) / 2.0;
        for (int i = 0; i < n; ++i) {
            const double bx = x0 + i * gap;
            if (i < n - 1) {
                addDone({ QPointF(bx, cy - bh / 2), QPointF(bx, cy + bh / 2) }, false);
            } else {
                m_closed = false;
                m_points << QPointF(bx, cy - bh / 2) << QPointF(bx, cy + bh / 2);
            }
        }
    } else if (type == "STOPLINE") {
        // 정지선: 한 줄 (두께는 도장 폭)
        m_closed = false;
        m_points << QPointF(cx - 1.6 * s, cy) << QPointF(cx + 1.6 * s, cy);
    } else if (type == "ARROW_F") {
        // 직진 화살표 = 기둥 한 획 + 화살촉 한 획.
        // 외곽선을 한 붓으로 두르면 붓 폭(50 mm)이 테두리로 나가서
        // 실제 도로 화살표가 아니라 '속 빈 도형'이 된다.
        addDone({ QPointF(cx,            cy - 1.05 * s),
                  QPointF(cx + 0.50 * s, cy - 0.50 * s) }, false);
        m_closed = false;
        m_points << QPointF(cx,            cy + 1.15 * s)
                 << QPointF(cx,            cy - 1.05 * s)
                 << QPointF(cx - 0.50 * s, cy - 0.50 * s);
    } else if (type == "ARROW_L" || type == "ARROW_R") {
        // 좌/우회전 화살표 = 「직진 → 회전 → 직진」 기둥 한 획 + 화살촉 한 획.
        // 로봇 동작(MOVE/TURN)과 1:1로 대응한다.
        const double m = (type == "ARROW_L") ? 1.0 : -1.0;
        addDone({ QPointF(cx - m * 0.55 * s, cy - 0.62 * s),
                  QPointF(cx - m * 1.00 * s, cy - 0.15 * s),
                  QPointF(cx - m * 0.55 * s, cy + 0.32 * s) }, false);
        m_closed = false;
        m_points << QPointF(cx + m * 0.15 * s, cy + 1.15 * s)
                 << QPointF(cx + m * 0.15 * s, cy - 0.15 * s)
                 << QPointF(cx - m * 1.00 * s, cy - 0.15 * s);
    } else if (type == "PARKING") {
        m_closed = false;
        m_points << QPointF(cx - 0.9 * s, cy - 1.1 * s)
                 << QPointF(cx - 0.9 * s, cy + 1.1 * s)
                 << QPointF(cx + 0.9 * s, cy + 1.1 * s)
                 << QPointF(cx + 0.9 * s, cy - 1.1 * s);
    } else if (type == "ZIGZAG") {
        m_closed = false;
        m_points << QPointF(cx - 1.2 * s, cy + 0.5 * s)
                 << QPointF(cx - 0.6 * s, cy - 0.5 * s)
                 << QPointF(cx,            cy + 0.5 * s)
                 << QPointF(cx + 0.6 * s, cy - 0.5 * s)
                 << QPointF(cx + 1.2 * s, cy + 0.5 * s);
    } else {
        m_closed = false;
        m_points << QPointF(cx - s, cy) << QPointF(cx + s, cy);
    }
    resetSelection();
    emit selectionChanged();
    emitPath();
    update();
}

// 글자 → 획 폴리라인 (결과는 이미지 px).
//
// 🔴 2026-07-29 방식 교체: 폰트 렌더 + 세선화를 버리고 **스트로크 폰트**를 쓴다.
//    (strokefont.h — 글자를 선/호로 직접 정의)
//
//    예전 방식은 굵게 렌더한 글자를 Zhang-Suen 으로 깎아 중심선을 뽑았는데,
//    획이 만나는 곳(A 의 가로대, K·X 의 교차점)에서 잔가지가 생기고 모서리가
//    둥글게 뭉개졌다. 실측: 'A' 한 글자가 20점짜리 덩어리 → 서버로 MOVE 260개.
//    'DA' 두 글자가 81 동작 · 31 회전이 됐다. 잡음을 걸러도 원본이 이미 3획이
//    아니라서 못 고친다.
//
//    지금은 'A' = 선 3개, 'W' = 선 4개, 'O' = 원 하나로 **정의 그대로** 나온다.
QList<QVector<QPointF>> VideoView::textStrokePolylines(const QString &text,
                                                       double targetHpx) const
{
    QList<QVector<QPointF>> out;
    if (text.isEmpty() || targetHpx < 4.0) return out;

    double emW = 1.0;
    const QList<strokefont::Stroke> strokes = strokefont::layout(text, &emW);
    if (strokes.isEmpty()) return out;

    // em 박스(가로 emW · 세로 1) → 목표 높이. y 는 폰트가 위로 +1 이고
    // 이미지 좌표는 아래로 + 이므로 뒤집는다.
    const double sc = targetHpx;
    const double cx = m_frame.isNull() ? 0.0 : m_frame.width() * 0.5;
    const double cy = m_frame.isNull() ? 0.0 : m_frame.height() * 0.5;
    const double ox = cx - emW * sc * 0.5;
    const double oy = cy + sc * 0.5;

    for (const strokefont::Stroke &st : strokes) {
        if (st.size() < 2) continue;
        QVector<QPointF> pts;
        pts.reserve(st.size());
        for (const QPointF &p : st)
            pts.append(QPointF(ox + p.x() * sc, oy - p.y() * sc));
        out.append(pts);
    }
    return out;
}

// 글자 도장. 기본은 획(중심선) — 붓 폭 50 mm 로 한 번에 칠해지는 형태.
void VideoView::addTextWorld(const QString &text, double heightMm, bool outline)
{
    pushUndo();
    if (!m_isTopView || text.trimmed().isEmpty() || m_frame.isNull()) return;
    stashActive();
    m_focused = true;              // 방금 넣은 것은 바로 편집할 수 있어야 한다
    m_drawing = false;

    const double pxPerMm = m_tvPxPerM / 1000.0;
    const double targetH = std::max(20.0, heightMm) * pxPerMm;

    if (!outline) {
        const QList<QVector<QPointF>> strokes = textStrokePolylines(text.trimmed(), targetH);
        for (const QVector<QPointF> &s : strokes) {
            VVPath dp;
            dp.closed = false;          // 획은 열린 경로 = 직진/회전으로만 주행
            dp.pts = s;
            m_done.append(dp);
        }
        if (!m_done.isEmpty()) {
            const VVPath last = m_done.takeLast();
            m_points = last.pts;
            m_closed = last.closed;
        }
    } else {
        QFont f(QStringLiteral("Pretendard"));
        f.setPixelSize(220);
        f.setWeight(QFont::Bold);
        QPainterPath pp;
        pp.addText(0, 0, f, text.trimmed());
        const QRectF br = pp.boundingRect();
        if (br.width() < 1 || br.height() < 1) return;
        const double sc = targetH / br.height();
        const double cx = m_frame.width() * 0.5, cy = m_frame.height() * 0.5;
        const double eps = std::max(1.0, strokePx() * 0.12);

        for (const QPolygonF &poly : pp.toSubpathPolygons()) {
            QVector<QPointF> pts;
            for (const QPointF &pt : poly)
                pts.append(QPointF(cx + (pt.x() - br.center().x()) * sc,
                                   cy + (pt.y() - br.center().y()) * sc));
            const QVector<QPointF> simp = rdpSimplify(pts, eps);
            if (simp.size() >= 3)
                m_done.append({ simp, true });
        }
    }
    resetSelection();
    syncDoneSelSize();   // 글자 획이 통째로 늘었다 — m_doneSel 길이를 맞춰 둔다
    emit selectionChanged();
    emitPath();
    update();
}

// 활성 도형의 거의 일직선인 점들을 합친다
int VideoView::simplifyActive(double toleranceMm)
{
    pushUndo();
    if (m_points.size() < 3) return 0;
    const double eps = std::max(0.5, toleranceMm / 1000.0 * m_tvPxPerM);
    const int before = m_points.size();
    m_points = rdpSimplify(m_points, eps);
    if (m_points.size() < 3) m_closed = false;
    resetSelection();
    emitPath();
    update();
    return before - m_points.size();
}

int VideoView::simplifyAll(double toleranceMm)
{
    pushUndo();
    const double eps = std::max(0.5, toleranceMm / 1000.0 * m_tvPxPerM);
    int removed = simplifyActive(toleranceMm);
    for (VVPath &dp : m_done) {
        if (dp.pts.size() < 3) continue;
        const int before = dp.pts.size();
        dp.pts = rdpSimplify(dp.pts, eps);
        if (dp.pts.size() < 3) dp.closed = false;
        removed += before - dp.pts.size();
    }
    emitPath();
    update();
    return removed;
}

void VideoView::setOverlayPaths(const QList<QList<QPointF>> &originalPx, const QList<bool> &closed)
{
    m_overlayPaths.clear();
    for (int k = 0; k < originalPx.size(); ++k) {
        VVPath op;
        op.closed = closed.value(k, false);
        op.pts = QVector<QPointF>(originalPx[k].begin(), originalPx[k].end());
        m_overlayPaths.append(op);
    }
    update();
}

void VideoView::emitPath()
{
    if (m_isTopView) emit pathChanged();
}

// ── 호모그래피(원본 px → topview px) ─────────────────────────────────
bool VideoView::parseHomography3x3(const QJsonArray &a, cv::Mat &out) const
{
    if (a.size() != 3) return false;
    out = cv::Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        QJsonArray row = a.at(r).toArray();
        if (row.size() != 3) return false;
        for (int c = 0; c < 3; ++c)
            out.at<double>(r, c) = row.at(c).toDouble();
    }
    return true;
}

void VideoView::configureTopViewTest()
{
    m_tvFrac = { QPointF(0.20, 0.50), QPointF(0.80, 0.50),
                 QPointF(0.98, 0.95), QPointF(0.02, 0.95) };
    m_tvRealW = 4.0; m_tvRealH = 3.0; m_tvPxPerM = 120.0;
    m_tvOutW = int(m_tvRealW * m_tvPxPerM);
    m_tvOutH = int(m_tvRealH * m_tvPxPerM);
    m_tvUseFrac = true;
    m_tvBuilt = false;
    m_hCoordUndistort = false;   // 테스트 보정에는 렌즈 모델이 없다
    const double ppm = m_tvPxPerM / 1000.0;
    const double maxYmm = m_tvRealH * 1000.0;
    m_mmToTv = cv::Mat::eye(3, 3, CV_64F);
    m_mmToTv.at<double>(0, 0) = ppm;
    m_mmToTv.at<double>(1, 1) = -ppm;
    m_mmToTv.at<double>(1, 2) = maxYmm * ppm;
    m_hasMmToTv = true;
    m_calibSummary = QStringLiteral("테스트 보정 (좌하 0,0 · 4.0×3.0 m)");
    emit scaleChanged();
}

// ── H 의 출력 단위 검산 ───────────────────────────────────────────────
//
// 왜 필요한가: 번들의 `unit` 필드를 믿을 수 없다. 서버(router.cpp:419)는
// 수신 즉시 H_floor/H_marker 를 ÷1000 해서 **미터로 바꿔** 저장·중계하는데
// `unit` 은 "mm" 그대로 남긴다. 그 말을 믿으면 좌표가 통째로 1000 배 어긋난다.
//
// 어떻게: 번들에 `canvas_mm` 과 `image_size` 가 이미 있다. 캔버스가 화면을
// 대충 채운다고 보면 "월드단위/픽셀" 배율의 기대값을 알 수 있고, 실측 배율이
// 기대값의 1 배면 mm, 1/1000 배면 m 이다. 두 후보가 **3 decade** 떨어져 있어서
// "대충 채운다"는 가정이 30 배 틀려도 판정이 뒤집히지 않는다.
//
// (실측: 현장 번들 2026-07-29-1411 은 r=3.04 → mm. 같은 H 를 ÷1000 하면
//  r=0.0030 → m. 판정 경계는 r=0.032 이라 양쪽 다 10 배 여유가 있다)
enum class HUnit { Unknown, Mm, M };

static HUnit verifyHUnit(const cv::Mat &Hraw, const QJsonObject &calib, double *ratioOut)
{
    if (Hraw.empty() || Hraw.type() != CV_64F) return HUnit::Unknown;
    const QJsonArray canvas = calib.value(QStringLiteral("canvas_mm")).toArray();
    const QJsonArray isz    = calib.value(QStringLiteral("image_size")).toArray();
    if (canvas.size() < 2 || isz.size() < 2) return HUnit::Unknown;
    const double cw = canvas.at(0).toDouble(), chh = canvas.at(1).toDouble();
    const double iw = isz.at(0).toDouble(),    ih  = isz.at(1).toDouble();
    if (cw < 10.0 || chh < 10.0 || iw < 16.0 || ih < 16.0) return HUnit::Unknown;

    const double *h = Hraw.ptr<double>();
    auto mapPt = [&](double x, double y, QPointF *out) {
        const double w = h[6] * x + h[7] * y + h[8];
        if (std::abs(w) < 1e-12) return false;
        *out = QPointF((h[0] * x + h[1] * y + h[2]) / w,
                       (h[3] * x + h[4] * y + h[5]) / w);
        return true;
    };

    // 영상 중심 부근의 국소 배율. 화면 끝은 원근 때문에 발산할 수 있어 쓰지 않는다.
    const double cx = iw * 0.5, cy = ih * 0.5;
    const double d = std::max(8.0, iw * 0.05);
    QPointF p0, pdx, pdy;
    if (!mapPt(cx, cy, &p0) || !mapPt(cx + d, cy, &pdx) || !mapPt(cx, cy + d, &pdy))
        return HUnit::Unknown;
    const double sX = QLineF(p0, pdx).length() / d;
    const double sY = QLineF(p0, pdy).length() / d;
    const double s = std::sqrt(std::max(1e-30, sX * sY));   // 기하평균
    const double expect = std::hypot(cw, chh) / std::hypot(iw, ih);   // mm/px 기대값
    if (s < 1e-12 || expect < 1e-9) return HUnit::Unknown;

    const double r = s / expect;      // 1 근처 = mm, 0.001 근처 = m
    if (ratioOut) *ratioOut = r;
    const double lg = std::log10(r);
    const double dMm = std::abs(lg);
    const double dM  = std::abs(lg + 3.0);
    if (std::min(dMm, dM) > 1.5) return HUnit::Unknown;   // 어느 쪽도 아니다 → 판정 보류
    return (dMm <= dM) ? HUnit::Mm : HUnit::M;
}

bool VideoView::configureTopViewCalib(const QJsonObject &calib)
{
    // H 가 어느 픽셀 공간을 받는지 기억해 둔다 — 원본 뷰로 되돌릴 때 왜곡을
    // 한 번 더 태울지가 여기서 갈린다.
    m_hCoordUndistort = (calib.value(QStringLiteral("coord_mode")).toString().toLower()
                         == QLatin1String("undistort"));
    // ── 단위 결정 ────────────────────────────────────────────────────
    // 선언값(`unit`)을 먼저 읽고, H 로 검산해서 어긋나면 **검산을 따른다**.
    // 선언을 그대로 믿던 예전 코드는 서버 경유 번들(H는 미터·unit은 "mm")에서
    // 좌표를 1000 배 축소시켰다.
    const QString unit = calib.value(QStringLiteral("unit")).toString(QStringLiteral("mm")).toLower();
    const bool declaredMm = (unit != QStringLiteral("m") && unit != QStringLiteral("meter")
                             && unit != QStringLiteral("meters"));

    // H 를 스케일 적용 **전에** 먼저 뜬다 (검산 대상이 원본 H 라야 한다).
    cv::Mat Hraw;
    bool haveH = false;
    if (parseHomography3x3(calib.value(QStringLiteral("H")).toArray(), Hraw))
        haveH = true;
    else if (parseHomography3x3(calib.value(QStringLiteral("H_floor")).toArray(), Hraw))
        haveH = true;

    bool unitIsMm = declaredMm;
    m_calibUnitNote.clear();
    if (haveH) {
        double ratio = 0.0;
        const HUnit v = verifyHUnit(Hraw, calib, &ratio);
        if (v == HUnit::Unknown) {
            m_calibUnitNote = QStringLiteral("단위 %1 (선언값 · 검산 불가)")
                                  .arg(declaredMm ? QStringLiteral("mm") : QStringLiteral("m"));
        } else {
            const bool verified = (v == HUnit::Mm);
            if (verified == declaredMm) {
                m_calibUnitNote = QStringLiteral("단위 %1 (검산 일치, r=%2)")
                                      .arg(verified ? QStringLiteral("mm") : QStringLiteral("m"))
                                      .arg(ratio, 0, 'g', 3);
            } else {
                // 🔴 선언과 실제가 다르다 — 조용히 넘기면 1000 배 사고다.
                unitIsMm = verified;
                m_calibUnitNote = QStringLiteral("⚠️ 단위 불일치: 선언 %1 / 실제 %2 "
                                                 "— 실제값 채택 (r=%3)")
                                      .arg(declaredMm ? QStringLiteral("mm") : QStringLiteral("m"))
                                      .arg(verified ? QStringLiteral("mm") : QStringLiteral("m"))
                                      .arg(ratio, 0, 'g', 3);
                qWarning().noquote() << "[VideoView]" << m_calibUnitNote;
            }
        }
    }
    cv::Mat Himg2mm;
    if (haveH) {
        Himg2mm = Hraw;
        if (!unitIsMm) {
            cv::Mat Sm = cv::Mat::eye(3, 3, CV_64F);
            Sm.at<double>(0, 0) = 1000.0;
            Sm.at<double>(1, 1) = 1000.0;
            Himg2mm = Sm * Himg2mm;
        }
    }

    QJsonArray corners = calib.value(QStringLiteral("corners")).toArray();
    if (corners.isEmpty() && calib.contains(QStringLiteral("corner")))
        corners = calib.value(QStringLiteral("corner")).toArray();

    std::vector<cv::Point2f> srcPx, dstMm;
    for (const QJsonValue &cv : corners) {
        const QJsonObject o = cv.toObject();
        QJsonArray px = o.value(QStringLiteral("px")).toArray();
        if (px.size() < 2) px = o.value(QStringLiteral("pixel")).toArray();
        if (px.size() < 2) px = o.value(QStringLiteral("raw")).toArray();
        QJsonArray mm = o.value(QStringLiteral("mm")).toArray();
        if (mm.size() < 2) mm = o.value(QStringLiteral("world")).toArray();
        if (mm.size() < 2 && o.contains(QStringLiteral("x"))) {
            mm = QJsonArray{ o.value(QStringLiteral("x")), o.value(QStringLiteral("y")) };
        }
        if (px.size() < 2 && o.contains(QStringLiteral("px_x")))
            px = QJsonArray{ o.value(QStringLiteral("px_x")), o.value(QStringLiteral("px_y")) };
        if (mm.size() < 2 && o.contains(QStringLiteral("mm_x")))
            mm = QJsonArray{ o.value(QStringLiteral("mm_x")), o.value(QStringLiteral("mm_y")) };
        if (px.size() < 2 || mm.size() < 2) continue;
        srcPx.emplace_back(float(px.at(0).toDouble()), float(px.at(1).toDouble()));
        // ⚠️ 앵커 좌표는 **항상 mm** 다 (키 이름이 곧 단위 — `mm`/`mm_x`). unit 은 H 의
        //    출력 단위일 뿐이라 여기에 곱하면 안 된다. 서버가 unit:"m" 로 내려주기
        //    시작한 뒤(2026-07-30) 앵커까지 ×1000 되던 잠재 버그를 막는다.
        dstMm.emplace_back(float(mm.at(0).toDouble()), float(mm.at(1).toDouble()));
    }

    auto buildMmToTv = [](double originX, double originY, double maxY, double pxPerMm) {
        cv::Mat M = cv::Mat::eye(3, 3, CV_64F);
        M.at<double>(0, 0) = pxPerMm;
        M.at<double>(1, 1) = -pxPerMm;
        M.at<double>(0, 2) = -originX * pxPerMm;
        M.at<double>(1, 2) = maxY * pxPerMm;
        Q_UNUSED(originY);
        return M;
    };

    if (srcPx.size() >= 4) {
        double maxAx = dstMm[0].x, maxAy = dstMm[0].y;
        for (const auto &p : dstMm) {
            maxAx = std::max(maxAx, double(p.x));
            maxAy = std::max(maxAy, double(p.y));
        }

        double originX = 0.0, originY = 0.0;
        const QJsonArray origin = calib.value(QStringLiteral("origin_mm")).toArray();
        if (origin.size() >= 2) {
            originX = origin.at(0).toDouble();
            originY = origin.at(1).toDouble();
        }

        double canvasW = 0.0, canvasH = 0.0;
        const QJsonArray canvas = calib.value(QStringLiteral("canvas_mm")).toArray();
        if (canvas.size() >= 2) {
            canvasW = canvas.at(0).toDouble();
            canvasH = canvas.at(1).toDouble();
        }
        if (canvasW < 10.0) canvasW = maxAx + std::max(40.0, 0.08 * maxAx);
        if (canvasH < 10.0) canvasH = maxAy + std::max(40.0, 0.08 * maxAy);
        canvasW = std::max(canvasW, maxAx - originX + 20.0);
        canvasH = std::max(canvasH, maxAy - originY + 20.0);

        const double worldWmm = canvasW;
        const double worldHmm = canvasH;
        const double maxY = originY + worldHmm;

        const double targetShort = 560.0;
        double pxPerMm = targetShort / std::min(worldWmm, worldHmm);
        if (calib.contains(QStringLiteral("px_per_mm")))
            pxPerMm = calib.value(QStringLiteral("px_per_mm")).toDouble(pxPerMm);
        pxPerMm = std::max(0.2, std::min(pxPerMm, 8.0));

        m_tvPxPerM = pxPerMm * 1000.0;
        m_tvOutW = qBound(240, int(std::ceil(worldWmm * pxPerMm)), 2000);
        m_tvOutH = qBound(240, int(std::ceil(worldHmm * pxPerMm)), 2000);
        m_tvRealW = worldWmm / 1000.0;
        m_tvRealH = worldHmm / 1000.0;

        m_mmToTv = buildMmToTv(originX, originY, maxY, pxPerMm);
        m_hasMmToTv = true;

        std::vector<cv::Point2f> dstTv;
        for (const auto &p : dstMm) {
            dstTv.emplace_back(float((p.x - originX) * pxPerMm),
                               float((maxY - p.y) * pxPerMm));
        }

        m_tvH = cv::findHomography(srcPx, dstTv, 0);
        QString note;

        // coord_mode 가 붙어 있으면 규격(QT-REQ-CCTV-001)에 맞춰 발행된 정식 번들이다.
        // 그 H 가 정답이므로 손으로 찍은 앵커와 겨루게 하지 않는다. 예전처럼 좋은 쪽을
        // 고르면, 앵커를 대충 찍은 날에 정식 H 가 밀려나 버린다.
        const bool hAuthoritative = haveH && calib.contains(QStringLiteral("coord_mode"));

        if (haveH && !m_tvH.empty()) {
            cv::Mat HfromServer = m_mmToTv * Himg2mm;
            auto rmse = [](const cv::Mat &H, const std::vector<cv::Point2f> &src,
                           const std::vector<cv::Point2f> &dst) {
                if (H.empty()) return 1e9;
                std::vector<cv::Point2f> pred;
                cv::perspectiveTransform(src, pred, H);
                double s = 0.0;
                for (size_t i = 0; i < dst.size(); ++i) {
                    const double dx = pred[i].x - dst[i].x;
                    const double dy = pred[i].y - dst[i].y;
                    s += dx * dx + dy * dy;
                }
                return std::sqrt(s / std::max<size_t>(1, dst.size()));
            };
            const double errCorner = rmse(m_tvH, srcPx, dstTv);
            const double errServer = rmse(HfromServer, srcPx, dstTv);
            if (hAuthoritative) {
                m_tvH = HfromServer;
                note = QStringLiteral(" · 번들H 채택(%1px, 앵커 %2px)")
                           .arg(errServer, 0, 'f', 1).arg(errCorner, 0, 'f', 1);
            } else if (errServer + 0.5 < errCorner) {
                m_tvH = HfromServer;
                note = QStringLiteral(" · H교차검증(서버채택 %1/%2px)")
                           .arg(errServer, 0, 'f', 1).arg(errCorner, 0, 'f', 1);
            } else {
                note = QStringLiteral(" · 앵커H(%1px) 서버H(%2px)")
                           .arg(errCorner, 0, 'f', 1).arg(errServer, 0, 'f', 1);
            }
        }

        if (m_tvH.empty() && haveH)
            m_tvH = m_mmToTv * Himg2mm;
        if (m_tvH.empty()) {
            configureTopViewTest();
            return false;
        }
        m_tvHinv = m_tvH.inv();
    m_tvMapDirty = true;   // H 가 바뀌면 remap 대응표도 다시 만들어야 한다
        m_tvUseFrac = false;
        m_tvBuilt = true;
        if (!m_calibUnitNote.isEmpty())
            note += QStringLiteral(" · ") + m_calibUnitNote;
        m_calibSummary = QStringLiteral("앵커%1 · 좌하(0,0) · %2×%3 mm · %4×%5px · 1px=%6mm%7")
                             .arg(int(srcPx.size()))
                             .arg(worldWmm, 0, 'f', 0)
                             .arg(worldHmm, 0, 'f', 0)
                             .arg(m_tvOutW)
                             .arg(m_tvOutH)
                             .arg(mmPerPx(), 0, 'f', 3)
                             .arg(note);
        emit scaleChanged();
        return true;
    }

    if (haveH) {
        double canvasW = 1200.0, canvasH = 1200.0;
        const QJsonArray canvas = calib.value(QStringLiteral("canvas_mm")).toArray();
        if (canvas.size() >= 2) {
            canvasW = std::max(100.0, canvas.at(0).toDouble());
            canvasH = std::max(100.0, canvas.at(1).toDouble());
        }
        double pxPerMm = calib.value(QStringLiteral("px_per_mm")).toDouble(0);
        if (pxPerMm <= 0) {
            const double ppm = calib.value(QStringLiteral("px_per_m")).toDouble(0);
            pxPerMm = (ppm > 0) ? (ppm / 1000.0) : (560.0 / std::min(canvasW, canvasH));
        }
        const double originX = 0.0, originY = 0.0;
        const double maxY = originY + canvasH;

        m_tvPxPerM = pxPerMm * 1000.0;
        m_tvOutW = qBound(240, int(std::ceil(canvasW * pxPerMm)), 2000);
        m_tvOutH = qBound(240, int(std::ceil(canvasH * pxPerMm)), 2000);
        m_tvRealW = canvasW / 1000.0;
        m_tvRealH = canvasH / 1000.0;
        m_mmToTv = buildMmToTv(originX, originY, maxY, pxPerMm);
        m_hasMmToTv = true;
        m_tvH = m_mmToTv * Himg2mm;
        m_tvHinv = m_tvH.inv();
    m_tvMapDirty = true;   // H 가 바뀌면 remap 대응표도 다시 만들어야 한다
        m_tvUseFrac = false;
        m_tvBuilt = true;
        m_calibSummary = QStringLiteral("H만 · 좌하(0,0) · %1×%2 mm · %3×%4px · 1px=%5mm%6")
                             .arg(canvasW, 0, 'f', 0).arg(canvasH, 0, 'f', 0)
                             .arg(m_tvOutW).arg(m_tvOutH)
                             .arg(mmPerPx(), 0, 'f', 3)
                             .arg(m_calibUnitNote.isEmpty()
                                      ? QString()
                                      : QStringLiteral(" · ") + m_calibUnitNote);
        emit scaleChanged();
        return true;
    }

    configureTopViewTest();
    return false;
}

QPointF VideoView::worldMmToTopPx(double xMm, double yMm) const
{
    if (m_hasMmToTv && !m_mmToTv.empty()) {
        std::vector<cv::Point2f> in{ cv::Point2f(float(xMm), float(yMm)) }, out;
        cv::perspectiveTransform(in, out, m_mmToTv);
        return QPointF(out[0].x, out[0].y);
    }
    const double s = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
    return QPointF(xMm / 1000.0 * s, yMm / 1000.0 * s);
}

QPointF VideoView::topPxToWorldMm(const QPointF &img) const
{
    if (m_hasMmToTv && !m_mmToTv.empty()) {
        std::vector<cv::Point2f> in{ cv::Point2f(float(img.x()), float(img.y())) }, out;
        cv::perspectiveTransform(in, out, m_mmToTv.inv());
        return QPointF(out[0].x, out[0].y);
    }
    const double s = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
    return QPointF(img.x() / s * 1000.0, img.y() / s * 1000.0);
}

// 화면 좌표에서 가장 가까운 활성 경로 꼭짓점. 없으면 -1.
// 판정 반경은 마우스 클릭 때 쓰는 것과 같은 12px(화면 기준)이다.
int VideoView::vertexAtView(qreal viewX, qreal viewY) const
{
    if (m_points.isEmpty()) return -1;
    double sx, sy, ox, oy;
    displayTransform(sx, sy, ox, oy);
    const QPointF img = mapToImage(QPointF(viewX, viewY));
    double best = (sx > 0) ? 12.0 / sx : 12.0;
    int hit = -1;
    for (int i = 0; i < m_points.size(); ++i) {
        const double d = QLineF(img, m_points[i]).length();
        if (d <= best) { best = d; hit = i; }
    }
    return hit;
}

QPointF VideoView::vertexWorldMm(int index) const
{
    if (index < 0 || index >= m_points.size()) return QPointF();
    return topPxToWorldMm(m_points[index]);
}

bool VideoView::setVertexWorldMm(int index, double xMm, double yMm)
{
    if (index < 0 || index >= m_points.size()) return false;
    pushUndo();
    QPointF img = worldMmToTopPx(xMm, yMm);
    if (!m_frame.isNull()) {
        img.setX(qBound(0.0, img.x(), double(m_frame.width() - 1)));
        img.setY(qBound(0.0, img.y(), double(m_frame.height() - 1)));
    }
    m_points[index] = img;
    emitPath();
    update();
    return true;
}

void VideoView::appendWorldPointMm(double xMm, double yMm)
{
    pushUndo();
    if (!m_isTopView) return;
    if (!m_drawing) {
        m_drawing = true;
        m_closed = false;
    }
    QPointF img = worldMmToTopPx(xMm, yMm);
    if (!m_frame.isNull()) {
        img.setX(qBound(0.0, img.x(), double(m_frame.width() - 1)));
        img.setY(qBound(0.0, img.y(), double(m_frame.height() - 1)));
    }
    if (!m_points.isEmpty() && QLineF(m_points.last(), img).length() <= mergeThresholdPx()) {
        emit pointsMerged(1);
        return;
    }
    m_points.append(img);
    emitPath();
    update();
}

void VideoView::addRectWorldMm(double widthMm, double heightMm)
{
    pushUndo();
    if (!m_isTopView || widthMm < 1.0 || heightMm < 1.0) return;
    stashActive();
    const double cxPx = m_tvOutW * 0.5;
    const double cyPx = m_tvOutH * 0.5;
    double cxMm = cxPx, cyMm = cyPx;
    if (m_hasMmToTv && !m_mmToTv.empty()) {
        cv::Mat inv = m_mmToTv.inv();
        std::vector<cv::Point2f> in{ cv::Point2f(float(cxPx), float(cyPx)) }, out;
        cv::perspectiveTransform(in, out, inv);
        cxMm = out[0].x; cyMm = out[0].y;
    } else {
        const double s = (m_tvPxPerM > 0) ? m_tvPxPerM : 100.0;
        cxMm = cxPx / s * 1000.0;
        cyMm = cyPx / s * 1000.0;
    }
    const double hx = widthMm * 0.5, hy = heightMm * 0.5;
    m_closed = true;
    m_drawing = false;
    m_points << worldMmToTopPx(cxMm - hx, cyMm - hy)
             << worldMmToTopPx(cxMm + hx, cyMm - hy)
             << worldMmToTopPx(cxMm + hx, cyMm + hy)
             << worldMmToTopPx(cxMm - hx, cyMm + hy);
    emitPath();
    update();
}

void VideoView::buildTopViewIfNeeded(int fw, int fh)
{
    if (!m_tvUseFrac || m_tvBuilt || fw <= 0 || fh <= 0) return;
    std::vector<cv::Point2f> src, dst;
    for (const QPointF &f : std::as_const(m_tvFrac))
        src.emplace_back(float(f.x() * fw), float(f.y() * fh));
    dst = {
        cv::Point2f(0.f, 0.f),
        cv::Point2f(float(m_tvOutW), 0.f),
        cv::Point2f(float(m_tvOutW), float(m_tvOutH)),
        cv::Point2f(0.f, float(m_tvOutH))
    };
    m_tvH = cv::findHomography(src, dst);
    if (!m_tvH.empty()) {
        m_tvHinv = m_tvH.inv();
        m_tvMapDirty = true;   // H 가 바뀌면 remap 대응표도 다시 만들어야 한다
        m_tvBuilt = true;
    }
}

QImage VideoView::warpToTopView(const QImage &src)
{
    if (src.isNull()) return QImage();
    buildTopViewIfNeeded(src.width(), src.height());
    if (!m_tvBuilt || m_tvH.empty()) return QImage();
    // ⚠️ 들어오는 프레임은 **BGR888** 이다 (video_worker::matToQImage 주석 참고).
    //    여기서 RGB888 로 바꾸면 워커에서 없앤 변환이 GUI 스레드로 옮겨올 뿐이다.
    //    remap 은 채널 순서를 신경쓰지 않으므로 BGR 그대로 펴서 BGR 로 내보낸다.
    QImage bgr = src.format() == QImage::Format_BGR888
                     ? src : src.convertToFormat(QImage::Format_BGR888);
    cv::Mat m(bgr.height(), bgr.width(), CV_8UC3,
              const_cast<uchar *>(bgr.bits()), size_t(bgr.bytesPerLine()));
    cv::Mat warped;

    // 렌즈 보정을 켜면 warpPerspective 대신 remap 을 쓴다.
    //
    // warpPerspective 는 호모그래피 한 장이라 **직선을 직선으로만** 보낸다. 렌즈가 만든
    // 곡률은 원리적으로 못 편다 — 그래서 보드 가운데는 맞는데 끝쪽이 휜다.
    // remap 은 TopView 픽셀 하나하나마다 "이 자리는 원본 사진의 어디냐"를 따로 묻기
    // 때문에 곡선도 정확히 되짚는다. 맵은 캘리브레이션이 바뀔 때만 다시 만든다.
    if (m_lensOn && m_cam.valid && m_cam.hasDistortion()) {
        buildTopViewRemapIfNeeded();
        if (!m_tvMapX.empty()) {
            cv::remap(m, warped, m_tvMapX, m_tvMapY, cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }
    }
    if (warped.empty())
        cv::warpPerspective(m, warped, m_tvH, cv::Size(m_tvOutW, m_tvOutH));

    QImage out(warped.data, warped.cols, warped.rows, int(warped.step), QImage::Format_BGR888);
    return out.copy();
}

// TopView 픽셀 → 원본 프레임 픽셀 대응표.
//
//   TopView px --(m_tvHinv)--> 왜곡보정 이미지 px --(정방향 왜곡)--> 원본 프레임 px
//
// m_tvHinv 는 이미 "TopView px → 이미지 px" 인데, 그 이미지 px 는 H 를 맞춘 공간
// (= 왜곡 보정된 픽셀) 이다. 그래서 마지막에 왜곡을 한 번 씌워야 실제 프레임 좌표가 된다.
// ⚠️ 원본 프레임은 절대 펴지 않는다. 펴는 건 이 TopView 렌더링뿐이다.
void VideoView::buildTopViewRemapIfNeeded()
{
    if (!m_tvMapDirty && !m_tvMapX.empty()
        && m_tvMapX.cols == m_tvOutW && m_tvMapX.rows == m_tvOutH)
        return;
    if (m_tvHinv.empty() || m_tvOutW <= 0 || m_tvOutH <= 0) return;

    m_tvMapX.create(m_tvOutH, m_tvOutW, CV_32FC1);
    m_tvMapY.create(m_tvOutH, m_tvOutW, CV_32FC1);

    const cv::Matx33d Hi(m_tvHinv.at<double>(0,0), m_tvHinv.at<double>(0,1), m_tvHinv.at<double>(0,2),
                         m_tvHinv.at<double>(1,0), m_tvHinv.at<double>(1,1), m_tvHinv.at<double>(1,2),
                         m_tvHinv.at<double>(2,0), m_tvHinv.at<double>(2,1), m_tvHinv.at<double>(2,2));

    for (int y = 0; y < m_tvOutH; ++y) {
        float *mx = m_tvMapX.ptr<float>(y);
        float *my = m_tvMapY.ptr<float>(y);
        for (int x = 0; x < m_tvOutW; ++x) {
            const double w = Hi(2,0)*x + Hi(2,1)*y + Hi(2,2);
            if (std::abs(w) < 1e-12) { mx[x] = -1.0f; my[x] = -1.0f; continue; }
            const QPointF ud((Hi(0,0)*x + Hi(0,1)*y + Hi(0,2)) / w,
                             (Hi(1,0)*x + Hi(1,1)*y + Hi(1,2)) / w);
            const QPointF raw = m_cam.distort(ud);
            mx[x] = float(raw.x());
            my[x] = float(raw.y());
        }
    }
    m_tvMapDirty = false;
}

void VideoView::setLensModel(const camcalib::Model &m)
{
    m_cam = m;
    m_tvMapDirty = true;
    update();
}

void VideoView::setLensCorrection(bool on)
{
    if (m_lensOn == on) return;
    m_lensOn = on;
    m_tvMapDirty = true;
    update();
}

QList<QPointF> VideoView::topViewToOriginal(const QVector<QPointF> &pts, bool closeLoop) const
{
    QList<QPointF> out;
    if (pts.isEmpty() || m_tvHinv.empty()) return out;

    // ⚠️ 끝점 두 개만 옮기면 안 된다. TopView 에서 직선인 것이 원본에서는 원근과
    //    렌즈 때문에 휘고, 현장 실측으로 화면 대각선에서 84.5px 까지 벌어졌다.
    //    잘게 쪼개서 통째로 옮기면 0.6px 수준으로 떨어진다.
    QVector<QPointF> dense;
    const int n = pts.size();
    const int segs = closeLoop ? n : n - 1;
    dense.reserve(segs * 8 + 1);
    for (int i = 0; i < segs; ++i) {
        const QPointF a = pts[i];
        const QPointF b = pts[(i + 1) % n];
        const double len = QLineF(a, b).length();
        const int div = qBound(1, int(std::ceil(len / 24.0)), 32);
        for (int k = 0; k < div; ++k) {
            const double t = double(k) / double(div);
            dense.append(a + (b - a) * t);
        }
    }
    dense.append(closeLoop ? pts.first() : pts.last());

    std::vector<cv::Point2f> in, res;
    in.reserve(dense.size());
    for (const QPointF &p : dense) in.emplace_back(float(p.x()), float(p.y()));
    cv::perspectiveTransform(in, res, m_tvHinv);

    // m_tvHinv 가 내놓는 건 "H 가 기대하는 픽셀 공간"이다. 번들이 coord_mode =
    // undistort 면 그건 왜곡 보정된 픽셀이므로, 원본 RTSP 영상 위에 얹으려면
    // 정방향 왜곡을 한 번 더 태워야 실제 바닥 위치에 맞는다.
    const bool needDistort = m_hCoordUndistort && m_cam.valid && m_cam.hasDistortion();
    out.reserve(int(res.size()));
    for (const cv::Point2f &p : res) {
        const QPointF q(p.x, p.y);
        out.append(needDistort ? m_cam.distort(q) : q);
    }
    return out;
}

QList<QList<QPointF>> VideoView::overlayPathsForOriginal(QList<bool> *closedOut) const
{
    QList<QList<QPointF>> out;
    if (closedOut) closedOut->clear();
    // 닫는 변까지 이미 점으로 펼쳐 넣으므로 closed 는 항상 false 로 넘긴다.
    // (true 로 주면 받는 쪽이 마지막↔첫 점을 곧은 직선으로 한 번 더 그어 휘지 않는다)
    for (const VVPath &dp : m_done) {
        out.append(topViewToOriginal(dp.pts, dp.closed && dp.pts.size() > 2));
        if (closedOut) closedOut->append(false);
    }
    if (m_points.size() >= 2) {
        out.append(topViewToOriginal(m_points, m_closed && m_points.size() > 2));
        if (closedOut) closedOut->append(false);
    }
    return out;
}

QList<QPolygonF> VideoView::overlayBandsForOriginal() const
{
    QList<QPolygonF> out;
    if (m_tvHinv.empty()) return out;
    const double w = strokePx();
    if (w < 0.5) return out;

    // 폭은 TopView(펴진 바닥) 축척에서만 균일하다. 그래서 밴드 외곽선을 여기서
    // 만들고, 그 외곽선 점들을 원본 좌표로 옮긴다. 원본에서는 가까운 쪽이 굵고
    // 먼 쪽이 가늘게 — 실제로 보이는 대로 — 나온다.
    auto bandOf = [&](const QVector<QPointF> &pts, bool closed) {
        if (pts.size() < 2) return;
        QPainterPath src;
        src.moveTo(pts[0]);
        for (int i = 1; i < pts.size(); ++i) src.lineTo(pts[i]);
        if (closed && pts.size() > 2) src.closeSubpath();

        QPainterPathStroker st;
        st.setWidth(w);
        st.setCapStyle(Qt::RoundCap);
        st.setJoinStyle(Qt::RoundJoin);
        const QList<QPolygonF> rings = st.createStroke(src).toSubpathPolygons();
        for (const QPolygonF &ring : rings) {
            if (ring.size() < 3) continue;
            const QList<QPointF> mapped = topViewToOriginal(ring, true);
            if (mapped.size() >= 3) out.append(QPolygonF(mapped));
        }
    };

    for (const VVPath &dp : m_done) bandOf(dp.pts, dp.closed);
    if (m_points.size() >= 2) bandOf(m_points, m_closed);
    return out;
}

void VideoView::setOverlayBands(const QList<QPolygonF> &originalPx)
{
    m_overlayBands = originalPx;
    update();
}

QList<QPointF> VideoView::metersToOriginal(const QList<QPointF> &meters) const
{
    QVector<QPointF> tv;
    tv.reserve(meters.size());
    for (const QPointF &m : meters)
        tv.append(worldMmToTopPx(m.x() * 1000.0, m.y() * 1000.0));
    return topViewToOriginal(tv);
}

QList<QList<QPointF>> VideoView::pathsToMeters(QList<bool> *closedOut) const
{
    QList<QList<QPointF>> out;
    if (closedOut) closedOut->clear();

    cv::Mat inv;
    const bool useMat = (m_hasMmToTv && !m_mmToTv.empty());
    if (useMat) inv = m_mmToTv.inv();

    auto convert = [&](const QVector<QPointF> &pts) {
        QList<QPointF> res;
        if (pts.isEmpty()) return res;
        if (useMat) {
            std::vector<cv::Point2f> in, mm;
            for (const QPointF &p : pts)
                in.emplace_back(float(p.x()), float(p.y()));
            cv::perspectiveTransform(in, mm, inv);
            for (const cv::Point2f &p : mm)
                res.append(QPointF(p.x / 1000.0, p.y / 1000.0));
        } else {
            const double s = (m_tvPxPerM > 0) ? m_tvPxPerM : 1.0;
            for (const QPointF &p : pts)
                res.append(QPointF(p.x() / s, (m_tvOutH - p.y()) / s));
        }
        return res;
    };

    for (const VVPath &dp : m_done) {
        if (dp.pts.size() < 2) continue;
        out.append(convert(dp.pts));
        if (closedOut) closedOut->append(dp.closed);
    }
    if (m_points.size() >= 2) {
        out.append(convert(m_points));
        if (closedOut) closedOut->append(m_closed);
    }
    return out;
}

void VideoView::setEditPathsMeters(const QList<QList<QPointF>> &metersPaths,
                                   const QList<bool> &closed)
{
    if (!m_isTopView) return;
    m_points.clear();
    m_closed = false;
    m_done.clear();
    m_drawing = false;
    m_hoverEdge = -1;
    m_focused = true;
    m_selection.clear();
    m_doneSel.clear();          // 도형을 통째로 갈아끼운다 — 옛 인덱스는 무효
    for (int k = 0; k < metersPaths.size(); ++k) {
        VVPath dp;
        dp.closed = closed.value(k, false) && metersPaths[k].size() > 2;
        for (const QPointF &m : metersPaths[k])
            dp.pts.append(worldMmToTopPx(m.x() * 1000.0, m.y() * 1000.0));
        if (dp.pts.size() >= 2)
            m_done.append(dp);
    }
    if (!m_done.isEmpty()) {
        const VVPath dp = m_done.takeLast();
        m_points = dp.pts;
        m_closed = dp.closed;
    }
    syncDoneSelSize();
    emit selectionChanged();
    emitPath();
    update();
}

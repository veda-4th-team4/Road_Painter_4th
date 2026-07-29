#ifndef VIDEOVIEW_H
#define VIDEOVIEW_H

#include <QQuickPaintedItem>
#include <QtQml/qqmlregistration.h>
#include <QImage>
#include <QVector>
#include <QPointF>
#include <QRectF>
#include <QList>
#include <QSet>
#include <QVariantList>
#include <QJsonObject>
#include <QTimer>
#include "camcalib.h"
#include "strokefont.h"
#include <QElapsedTimer>
#include <functional>
#include <opencv2/opencv.hpp>

// 하나의 그려진 경로 (다중 도형 지원용)
struct VVPath {
    QVector<QPointF> pts;   // 표시 프레임 px (TopView) 또는 원본 px (CCTV 오버레이)
    bool closed = false;
};

// 💡 [QML판] 영상 표시 + 작도 표면.
//   - topView=true: 보정된 바닥 위에서 경로 작도. topView=false: 원본 + 오버레이.
//   - 다중 도형: "활성 경로"(m_points) 하나를 편집하고, 완성본은 m_done 에 쌓인다.
//   - 뷰 조작(CAD/Figma 표준):
//       휠            = 커서 기준 확대/축소
//       휠클릭 드래그  = 화면 이동 (Alt+좌드래그도 동일)
//       좌클릭        = 점/도형 선택·이동 (편집이 최우선 — 확대 때문에 막히지 않음)
//       빈 곳 좌드래그 = 올가미(사각) 다중 선택
//       Ctrl/Shift+클릭 = 선택 토글
//   - 도장 폭(기본 50 mm)을 실제 두께로 그려 도포 면적을 그대로 보여준다.
class VideoView : public QQuickPaintedItem {
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool topView READ topView WRITE setTopView NOTIFY topViewChanged)
    Q_PROPERTY(bool interactive READ interactive WRITE setInteractive NOTIFY interactiveChanged)
    Q_PROPERTY(double mmPerPx READ mmPerPx NOTIFY scaleChanged)
    // 화면 1mm 가 몇 px 인지 (표시 배율 반영) — QML 축척 눈금자가 쓴다
    Q_PROPERTY(double screenPxPerMm READ screenPxPerMm NOTIFY scaleChanged)
    Q_PROPERTY(int zoomPercent READ zoomPercent NOTIFY scaleChanged)
    Q_PROPERTY(int selectionCount READ selectionCount NOTIFY selectionChanged)
    Q_PROPERTY(int undoDepth READ undoDepth NOTIFY pathChanged)
    Q_PROPERTY(bool hasActiveShape READ hasActiveShape NOTIFY selectionChanged)
    Q_PROPERTY(bool showLabels READ showLabels WRITE setShowLabels NOTIFY showLabelsChanged)
    Q_PROPERTY(double strokeWidthMm READ strokeWidthMm WRITE setStrokeWidthMm NOTIFY strokeWidthChanged)
    // 영역 확대 도구: 켜면 좌드래그가 "그 사각형에 맞춰 확대"가 된다
    Q_PROPERTY(bool zoomTool READ zoomTool WRITE setZoomTool NOTIFY zoomToolChanged)
public:
    explicit VideoView(QQuickItem *parent = nullptr);

    bool topView() const { return m_isTopView; }
    void setTopView(bool v);
    bool interactive() const { return m_interactive; }
    void setInteractive(bool v);

    void paint(QPainter *p) override;

    // C++/Backend 연동 (QML 노출 불필요)
    void configureTopViewTest();
    // 렌즈 보정 (K + 왜곡계수). TopView 배경을 remap 으로 펼 때만 쓴다 — 원본 프레임은
    // 건드리지 않는다. 끄면 예전처럼 warpPerspective 로 돌아간다 (비교용).
    void setLensModel(const camcalib::Model &m);
    void setLensCorrection(bool on);
    bool lensCorrection() const { return m_lensOn; }
    bool configureTopViewCalib(const QJsonObject &calib); // false = 실패(폴백)
    QVector<QPointF> currentPoints() const { return m_points; }
    bool isClosed() const { return m_closed; }
    int totalPointCount() const;
    int shapeCount() const;
    QList<QList<QPointF>> pathsToMeters(QList<bool> *closedOut = nullptr) const;
    QList<QList<QPointF>> overlayPathsForOriginal(QList<bool> *closedOut = nullptr) const;
    // 원본(왜곡) 뷰에 얹을 도포 폭 밴드. 원본에서는 폭이 위치마다 달라서 그쪽에서
    // 계산할 수 없다 → TopView 축척에서 폴리곤을 만든 뒤 원근+렌즈 왜곡을 태워 보낸다.
    QList<QPolygonF> overlayBandsForOriginal() const;
    void setOverlayPaths(const QList<QList<QPointF>> &originalPx, const QList<bool> &closed);
    void setOverlayBands(const QList<QPolygonF> &originalPx);
    QList<QPointF> metersToOriginal(const QList<QPointF> &meters) const;
    double pxPerMeter() const { return m_tvPxPerM; }
    double mmPerPx() const { return (m_tvPxPerM > 1e-9) ? 1000.0 / m_tvPxPerM : 0.0; }
    double screenPxPerMm() const;
    int zoomPercent() const;
    int selectionCount() const { return selectedPointCount(); }
    int undoDepth() const;
    bool hasActiveShape() const { return m_points.size() >= 2; }
    bool showLabels() const { return m_showLabels; }
    void setShowLabels(bool v);
    double strokeWidthMm() const { return m_strokeMm; }
    void setStrokeWidthMm(double mm);
    bool zoomTool() const { return m_zoomTool; }
    void setZoomTool(bool on);
    QString calibSummary() const { return m_calibSummary; }

    void setEditPathsMeters(const QList<QList<QPointF>> &metersPaths, const QList<bool> &closed);
    void setMarkerCorners(const QVariantList &cornersPx);
    void setArucoMarkers(const QList<int> &ids, const QList<QPolygonF> &cornersPx);
    void setArucoVisible(bool on);

    // 커밋된 미션 경로 (ghost + progress fill) / 로봇 POSE
    void setMissionPathsMeters(const QList<QList<QPointF>> &metersPaths, const QList<bool> &closed);
    void setMissionPathsPixels(const QList<QList<QPointF>> &imagePx, const QList<bool> &closed);
    void setMissionProgress(double progress01);
    void setRobotPose(double xM, double yM, double thetaDeg, bool valid);
    // 펜(노즐) 끝 위치. 회전중심 뒤 d 에 있어서 로봇 아이콘만 봐서는 어디를 칠하는지
    // 알 수 없다 — 꼭짓점 후진/제자리회전이 눈에 보이게 따로 찍어 준다.
    void setPenMarker(double xM, double yM, bool down, bool valid);
    void clearMission();

public slots:
    void onFrame(const QImage &original);
    void startDraw();
    void clearPath();
    void undo();
    void setPresetShape(const QString &type);
    Q_INVOKABLE void addPresetAt(const QString &type, qreal viewX, qreal viewY);
    Q_INVOKABLE void appendWorldPointMm(double xMm, double yMm);
    Q_INVOKABLE void addRectWorldMm(double widthMm, double heightMm);
    // 글자 → 붓이 지나갈 중심선(획). 외곽선이 아니라 세선화한 스켈레톤이라
    // "ㄷ" 은 직선 3구간이 된다. outline=true 면 예전처럼 외곽선.
    Q_INVOKABLE void addTextWorld(const QString &text, double heightMm, bool outline = false);
    Q_INVOKABLE void finishDraw();
    // 거의 일직선인 점들을 합쳐 "직진 + 회전"만 남긴다
    Q_INVOKABLE int simplifyActive(double toleranceMm);
    Q_INVOKABLE int simplifyAll(double toleranceMm);

    // 변 길이(mm) 수치 편집 — 활성 경로 대상
    Q_INVOKABLE int edgeAtView(qreal viewX, qreal viewY) const;
    Q_INVOKABLE double edgeLengthMm(int index) const;
    Q_INVOKABLE bool setEdgeLengthMm(int index, double mm);

    // 꼭짓점 월드좌표(mm) 조회/편집 — 활성 경로 대상.
    // 화면에서 눈대중으로 찍은 점을 실제 도면 수치로 바로잡을 때 쓴다.
    Q_INVOKABLE int vertexAtView(qreal viewX, qreal viewY) const;
    Q_INVOKABLE QPointF vertexWorldMm(int index) const;
    Q_INVOKABLE bool setVertexWorldMm(int index, double xMm, double yMm);

    // 꼭짓점 회전각(프로토콜 TURN, 양수 = 좌회전) 조회/편집
    Q_INVOKABLE int turnBadgeAtView(qreal viewX, qreal viewY) const;
    Q_INVOKABLE double turnAngleAt(int index) const;
    Q_INVOKABLE bool setTurnAngleAt(int index, double deg);

    // 뷰 조작
    Q_INVOKABLE void zoomAtView(qreal viewX, qreal viewY, qreal factor);
    Q_INVOKABLE void zoomBy(qreal factor);          // 화면 중앙 기준
    Q_INVOKABLE void setZoomPercent(int percent);   // 수치 직접 입력
    Q_INVOKABLE void zoomToViewRect(qreal x, qreal y, qreal w, qreal h);
    Q_INVOKABLE void fitView();                     // 배율/이동 초기화
    Q_INVOKABLE void zoomToSelection();             // 선택(없으면 활성 도형)에 맞춤

    // 변환 — 선택된 점이 있으면 선택만, 없으면 활성 도형 전체
    Q_INVOKABLE void rotateActive(double deg);
    Q_INVOKABLE void flipActive(bool horizontal);
    Q_INVOKABLE void scaleActive(double factor);
    Q_INVOKABLE void selectAllActive();
    Q_INVOKABLE void clearSelection();
    Q_INVOKABLE void deleteSelection();

signals:
    void topViewChanged();
    void interactiveChanged();
    void scaleChanged();
    void selectionChanged();
    void showLabelsChanged();
    void strokeWidthChanged();
    void zoomToolChanged();
    void pathChanged();
    void freeDrawEnded();
    void pathClosed();
    void pointsMerged(int removed);
    void edgeEditRequested(int index, double mm, qreal viewX, qreal viewY);
    void vertexEditRequested(int index, double xMm, double yMm, qreal viewX, qreal viewY);
    void turnEditRequested(int index, double deg, qreal viewX, qreal viewY);

protected:
    void mousePressEvent(QMouseEvent *) override;
    void mouseDoubleClickEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void hoverMoveEvent(QHoverEvent *) override;
    void wheelEvent(QWheelEvent *) override;
    void geometryChange(const QRectF &newGeom, const QRectF &oldGeom) override;

private:
    double viewPadding() const;
    double fitScale() const;                        // 배율 1.0 일 때의 스케일
    void displayTransform(double &sx, double &sy, double &ox, double &oy) const;
    void clampPan();
    QPointF mapToImage(const QPointF &pos) const;
    void buildTopViewIfNeeded(int fw, int fh);
    QImage warpToTopView(const QImage &src);
    // closeLoop = true 면 마지막↔첫 점 구간까지 점으로 펼쳐 넣는다.
    QList<QPointF> topViewToOriginal(const QVector<QPointF> &pts, bool closeLoop = false) const;
    void buildPreset(const QString &type, double cx, double cy);
    void emitPath();
    double strokePx() const;                        // 도장 폭(이미지 px)
    void paintBand(QPainter *p, const QVector<QPointF> &pts, bool closed,
                   double sx, double sy, double ox, double oy, const QColor &c);
    void paintMission(QPainter *p, double sx, double sy, double ox, double oy);
    void paintRobot(QPainter *p, double sx, double sy, double ox, double oy);
    void paintPenMarker(QPainter *p, double sx, double sy, double ox, double oy);
    void paintGrid(QPainter *p, double sx, double sy, double ox, double oy);
    void paintMarker(QPainter *p, double sx, double sy, double ox, double oy);
    void paintAruco(QPainter *p, double sx, double sy, double ox, double oy);
    void paintDonePaths(QPainter *p, double sx, double sy, double ox, double oy);
    void paintPathGuides(QPainter *p, const QVector<QPointF> &pts, bool closed,
                         double sx, double sy, double ox, double oy, bool full);
    void drawSegArrow(QPainter *p, const QPointF &aW, const QPointF &bW,
                      const QColor &fill, double t = 0.62);
    void paintEdgeLengths(QPainter *p, const QVector<QPointF> &pts, bool closed,
                          double sx, double sy, double ox, double oy);
    // 선택 영역 박스 + 크기조절/회전 핸들 (Figma·Illustrator 방식)
    QRectF selectionBoxImg() const;
    int handleAtView(const QPointF &v) const;
    void paintSelectionBox(QPainter *p, double sx, double sy, double ox, double oy);
    void applyHandleDrag(const QPointF &img);
    void commitDrawPoint(QPointF img);
    void finishFreeDraw();
    QPointF worldMmToTopPx(double xMm, double yMm) const;
    QPointF topPxToWorldMm(const QPointF &img) const;   // 위의 역변환
    bool parseHomography3x3(const QJsonArray &a, cv::Mat &out) const;
    double mergeThresholdPx() const;
    int mergeClosePoints();
    void stashActive();
    bool activateDoneAt(const QPointF &img, double thr);
    // 글자 렌더 → 세선화 → 스켈레톤 추적 (이미지 px 폴리라인)
    QList<QVector<QPointF>> textStrokePolylines(const QString &text, double targetHpx) const;
    // 변환 대상 인덱스 (선택 없으면 전체)
    QList<int> transformTargets() const;
    QPointF transformPivot() const;

    // ── 선택 (활성 + 완성 도형을 함께 다룬다) ────────────────────────
    int selectedPointCount() const;
    bool hasMultiShapeSelection() const;   // 완성 도형에도 선택이 걸려 있는가
    void clearDoneSelection();
    void syncDoneSelSize();
    // 선택을 활성·완성 도형 **양쪽에서** 지운다. 지운 게 있으면 true.
    // ⚠️ m_selection 만 비우면 완성 도형 쪽(m_doneSel)이 남아 파란 점이 화면에
    //    계속 박혀 있고, 다음 조작이 "안 보이는 선택"까지 끌고 간다.
    bool resetSelection();
    // 선택 전체를 끌어서 옮기기 시작한다 (활성 경로 + 완성 도형 함께).
    //   onVertex: 선택된 꼭짓점을 직접 집었는가 (아니면 박스 안쪽을 눌렀는가)
    void beginSelectionMove(const QPointF &img, bool onVertex);
    // 선택된 모든 점(없으면 활성 도형 전체)에 변환을 적용한다
    void applyToSelection(const std::function<QPointF(const QPointF &)> &fn);
    QRectF selectionBoundsImg() const;

    // ── 되돌리기 (동작 단위 스냅샷) ──────────────────────────────────
    struct Snapshot {
        QList<VVPath> done;
        QVector<QPointF> pts;
        bool closed = false;
        bool drawing = false;
    };
    void pushUndo();
    // 드래그는 "실제로 움직였을 때만" 되돌리기에 올린다.
    // 잡았다 놓기만 해도 스택이 쌓이면 Ctrl+Z 가 헛돈다.
    void beginPendingUndo();
    void commitPendingUndo();
    QList<Snapshot> m_undo;
    Snapshot m_pendingUndo;
    bool m_pendingUndoValid = false;

    bool m_isTopView = false;
    // TopView 워프 스로틀 — warpPerspective 는 GUI 스레드에서 돈다 (아래 onFrame 주석 참고)
    QElapsedTimer m_tvThrottle;
    // 렌즈 보정 — 켜져 있고 계수가 있을 때만 remap 경로를 탄다
    camcalib::Model m_cam;
    bool m_lensOn = false;
    bool m_tvMapDirty = true;
    cv::Mat m_tvMapX, m_tvMapY;
    void buildTopViewRemapIfNeeded();
    bool m_interactive = false;
    QImage m_frame;

    // 뷰 배율/이동
    double m_zoom = 1.0;
    double m_panX = 0.0, m_panY = 0.0;
    bool m_panning = false;
    QPointF m_panStartView;
    double m_panStartX = 0.0, m_panStartY = 0.0;

    // 작도 상태 — 활성 경로
    bool m_drawing = false;
    QVector<QPointF> m_points;
    bool m_closed = false;
    QList<VVPath> m_done;
    QPointF m_mouse;
    bool m_hover = false;
    int m_dragIdx = -1;
    int m_hoverEdge = -1;
    int m_hoverVertex = -1;

    // 다중 선택 / 올가미
    // m_selection = 활성 경로의 점, m_doneSel = 완성 도형별 점.
    // 올가미로 화면 전체를 긁으면 두 곳 모두 채워져야 "전체 선택"이 된다.
    QSet<int> m_selection;
    QList<QSet<int>> m_doneSel;
    bool m_rubber = false;
    bool m_zoomTool = false;    // 영역 확대 모드 (올가미 대신 확대)
    bool m_rubberZoom = false;  // 이번 드래그가 확대용인가
    QPointF m_rubberStart;      // 뷰 좌표
    QPointF m_rubberEnd;
    QVector<QPointF> m_dragStartPts;  // 선택 이동 기준
    QPointF m_dragStartImg;

    // Ctrl+드래그 비율 스케일링
    bool m_scaling = false;
    QPointF m_scaleAnchor;
    QVector<QPointF> m_scaleStart;
    double m_scaleStartDist = 0.0;

    // 도형 전체 이동
    bool m_movingShape = false;
    QPointF m_moveStartImg;
    QVector<QPointF> m_moveStartPts;

    // 선택 전체 이동 — 크기 조절(핸들)과 달리 완성 도형까지 같이 움직여야 해서
    // 활성 경로와 완성 도형의 시작 좌표를 따로 보관한다.
    bool m_movingSel = false;
    QPointF m_selMoveStartImg;
    QVector<QPointF> m_selMoveStartPts;           // 활성 경로
    QList<QVector<QPointF>> m_selMoveStartDone;   // 완성 도형별
    // 전체 선택 상태면 박스가 화면을 다 덮어서 "빈 곳 클릭 = 선택 해제"가 막힌다.
    // 실제로 끌었는지(dragged) / 점을 집었는지(onVertex) 를 구분해, 안쪽을 그냥
    // 클릭만 한 경우는 빈 곳 클릭으로 되돌린다.
    bool m_selMoveDragged = false;
    bool m_selMoveOnVertex = false;

    // 활성 도형의 편집 표시(선택 박스·핸들·치수 배지·꼭짓점)를 켤지.
    // 빈 곳을 클릭하면 꺼져서 완성 도형과 똑같이 보인다 — 도면을 다 그린 뒤에도
    // 박스와 배지가 화면에 계속 박혀 있어 결과를 못 보던 문제 때문에 넣었다.
    // 도형·점을 다시 클릭하면 켜진다. 데이터에는 영향이 없다(표시 전용).
    bool m_focused = true;

    // 표시 옵션
    bool m_showLabels = true;
    double m_strokeMm = 50.0;   // 로봇 도장 폭 — 고정값

    // 원본 뷰 오버레이 (원본 px)
    QList<VVPath> m_overlayPaths;
    QList<QPolygonF> m_overlayBands;   // 원본 뷰용 도포 폭 밴드 (원본 px)
    // H 가 왜곡 보정된 픽셀을 받는가 (번들의 coord_mode == "undistort").
    // 참이면 원본 영상에 얹을 때 정방향 왜곡을 한 번 더 태워야 한다.
    bool m_hCoordUndistort = false;
    QList<QPointF> m_markerPx;

    // ArUco 검출 결과 (원본 px)
    QList<int> m_arucoIds;
    QList<QPolygonF> m_arucoPolys;
    bool m_arucoVisible = true;

    // 호모그래피(원본 px → topview px) / 월드 mm ↔ topview
    bool m_tvBuilt = false;
    bool m_tvUseFrac = false;
    cv::Mat m_tvH, m_tvHinv;
    cv::Mat m_mmToTv;
    bool m_hasMmToTv = false;
    int m_tvOutW = 500, m_tvOutH = 500;
    double m_tvPxPerM = 100.0, m_tvRealW = 4.0, m_tvRealH = 3.0;
    QList<QPointF> m_tvFrac;
    QString m_calibSummary = "기본 테스트 보정";

    mutable QVector<QRectF> m_edgeLabelRects;
    mutable QVector<QRectF> m_turnBadgeRects;   // 꼭짓점별 회전각 배지 (뷰 좌표)
    mutable QVector<QRectF> m_handleRects;      // 선택 박스 핸들 (뷰 좌표)

    // 선택 박스 핸들 드래그 (0~3 모서리, 4~7 변, 8 회전)
    int m_handleIdx = -1;
    QRectF m_boxImg;
    QPointF m_handleAnchor;     // 반대편 고정점
    QPointF m_handleOrigin;     // 잡은 핸들의 원래 위치
    QPointF m_rotStartImg;
    QVector<QPointF> m_handleStart;
    // 드래그 시작 시점의 전체 좌표 (0=활성, 1..=완성 도형) — 누적 오차 방지
    QList<VVPath> m_handleStartAll;

    // 미션 오버레이 (표시 프레임 px)
    QList<VVPath> m_missionPaths;
    double m_missionProgress = 0.0;
    bool m_robotValid = false;
    bool m_penValid = false;
    bool m_penDown = false;
    QPointF m_penPx;
    QPointF m_robotPx;
    double m_robotThetaDeg = 0.0;

    QTimer *m_clickTimer = nullptr;
    bool m_pendingClick = false;
    QPointF m_pendingImg;
};

#endif // VIDEOVIEW_H

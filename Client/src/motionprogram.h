#pragma once
// ── 도면 폴리라인 → 서버에 전달할 논리 동작 시퀀스 ──────────────────────────
//
// Qt는 노즐이 바닥에 그려야 하는 도면 기하만 만든다. 서버 v2가 이 시퀀스를 로봇
// op으로 변환하면서 부호 반전, 펜 오프셋 이동, 노즐 타이밍, ARC 실행 반지름을 맡는다.
// Qt가 서버/로봇의 기구 보정을 미리 섞으면 이중 보정이 되므로 금지한다.
//
// ── 🔴 펜(노즐) 오프셋은 여기서 다루지 않는다 ──
// 예전에는 이 파일이 꼭짓점마다 "노즐 올림 → 후진 d → 회전 → 전진 d" 를 끼워 넣고
// 맨 앞에 lead-in MOVE +d 를 넣었다. 지금은 **전부 뺐다.**
//
//   근거: feature/server-driven-v2 Server/src/ops_builder.hpp.
//   Qt program은 도면 그대로이고, 서버가 buildDrawOps()에서 로봇 실행 op으로 바꾼다.
//
//   왜: 후진/재전진 거리는 '정확히 d' 여야 성립하는데, 그 정밀도를 가진 쪽(로봇의
//   스텝·모터)과 값을 만드는 쪽(Qt)이 분리돼 있으면 둘 다 손해다. 로봇은 이미
//   155mm 상수와 오프셋 이동 상태머신은 최신 서버 설정과 로봇 실행부의 책임이다.
//
// ── 🔴 속도도 프로토콜에 없다 ──
// Op::speed 는 **화면 미리보기와 예상 소요시간 계산용 로컬 값**이다. 전송하지
// 않는다 (SERVER_PROTOCOL: "속도는 프로토콜에 없다 — 전부 로봇 펌웨어 고정값").
//
// ── 🔴 노즐은 NOZZLE op 으로만 ──
// MOVE.paint / ARC.paint 는 "이 구간이 도색 구간인가" 표시일 뿐이고 노즐을
// 움직이는 값이 아니다. 도색 구간 **앞에 NOZZLE down, 뒤에 NOZZLE up** 을 Qt 가
// 직접 넣어야 한다. 경로는 노즐이 올라간 상태에서 시작하고 NOZZLE up 으로 끝난다.
#include <QLineF>
#include <QList>
#include <QPointF>
#include <QString>
#include <cmath>

namespace motionprogram {

// ⚠️ M_PI 를 쓰지 않는다 — MinGW <cmath> 는 _USE_MATH_DEFINES 없이는 정의하지
// 않아서, 이 헤더를 먼저 include 하는 번역 단위에서 컴파일이 깨진다.
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
// 서버·로봇팀이 모터 토크와 바깥 바퀴 SPS를 기준으로 확정한 도색 ARC 하한.
// 추후 LOGIN_OK가 min_paint_radius_m을 제공하면 이 상수 대신 서버 값을 사용한다.
constexpr double kServerConfirmedMinPaintRadiusM = 0.200;

// 로봇 속도. 도색 중에는 도료가 고르게 깔려야 해서 이동할 때보다 느리다.
// ⚠️ 전송하지 않는다 — 미리보기/예상시간 전용.
struct Speeds {
    double travelMps = 0.20;   // 칠하지 않고 이동할 때 (m/s)
    double paintMps = 0.10;    // 도색하며 주행할 때 (m/s)
    double turnDps = 30.0;     // 제자리 회전 (deg/s)
    double moveFor(bool painting) const { return painting ? paintMps : travelMps; }
};

struct Op {
    enum Kind { Move, Turn, Nozzle, Arc };
    Kind kind = Move;
    double dist = 0.0;     // m — Move 직진거리(음수=후진) / Arc 호 길이(항상 양수)
    double angle = 0.0;    // deg — Turn 회전각(양수=좌회전) / Arc 스윕각(항상 양수)
    double radius = 0.0;   // m — Arc 도면상 곡선 반지름 (양수)
    bool arcLeft = true;   // Arc — 좌회전(CCW)인가
    // 서버 v2 입력 기준의 도면 방위(CCW+): MOVE=진행 방향, TURN=회전 후 방향,
    // ARC=호 진입 접선. ARC 종료 방향은 heading +/- angle로 서버가 계산한다.
    double heading = 0.0;
    bool paint = false;    // Move/Arc — 이 구간을 칠하는가 (표시용, 노즐 제어 아님)
    bool down = false;     // Nozzle — 내리는가
    int vertex = 0;        // 이 op 가 "떠나는" 도면 꼭짓점 index (프로토콜 `v`, 필수)
    double speed = 0.0;    // 로컬 전용 — Move/Arc = m/s, Turn = deg/s. 전송 안 함

    QString opName() const
    {
        switch (kind) {
        case Turn:   return QStringLiteral("TURN");
        case Nozzle: return QStringLiteral("NOZZLE");
        case Arc:    return QStringLiteral("ARC");
        default:     return QStringLiteral("MOVE");
        }
    }
    // 프로토콜 direction 필드
    QString arcDirection() const { return arcLeft ? QStringLiteral("left")
                                                  : QStringLiteral("right"); }
};

inline double normDeg(double a)
{
    while (a > 180.0) a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

// ── 원호 검출 ────────────────────────────────────────────────────────────
// 'D', 'O', 루프 프리셋처럼 곡선을 잘게 쪼갠 폴리라인을 ARC 한 개로 되돌린다.
// (SERVER_PROTOCOL §ARC op 규약 — 2026-07-29 신설)
//
// 보수적으로 잡는다: 조건을 하나라도 못 맞추면 예전처럼 MOVE/TURN 으로 나간다.
// 직선 위주 작업(횡단보도·정지선)이 이 코드 때문에 달라질 일은 없다.
namespace detail {

struct Circle { QPointF c; double r = 0.0; bool ok = false; };

// Kåsa 대수적 원 피팅 — x²+y²+Dx+Ey+F=0 를 최소제곱으로 푼다.
inline Circle fitCircle(const QList<QPointF> &pts, int a, int b)
{
    Circle out;
    const int n = b - a + 1;
    if (n < 3) return out;
    // 월드 원점에서 멀리 떨어진 큰 도형도 안정적으로 풀도록 평균점을 원점으로 옮긴다.
    // 원래 좌표를 그대로 정규방정식에 넣으면 x²/y² 항이 커져 위치에 따라 피팅 결과가
    // 달라질 수 있다.
    double mx = 0.0, my = 0.0;
    for (int i = a; i <= b; ++i) {
        mx += pts[i].x();
        my += pts[i].y();
    }
    mx /= n;
    my /= n;
    double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0, sz = 0;
    for (int i = a; i <= b; ++i) {
        const double x = pts[i].x() - mx, y = pts[i].y() - my;
        const double z = x * x + y * y;
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
        sxz += x * z; syz += y * z; sz += z;
    }
    const double N = n;
    // 정규방정식 3x3
    const double a11 = sxx, a12 = sxy, a13 = sx;
    const double a21 = sxy, a22 = syy, a23 = sy;
    const double a31 = sx,  a32 = sy,  a33 = N;
    const double b1 = -sxz, b2 = -syz, b3 = -sz;
    const double det = a11 * (a22 * a33 - a23 * a32)
                     - a12 * (a21 * a33 - a23 * a31)
                     + a13 * (a21 * a32 - a22 * a31);
    if (std::abs(det) < 1e-18) return out;
    const double D = (b1 * (a22 * a33 - a23 * a32)
                    - a12 * (b2 * a33 - a23 * b3)
                    + a13 * (b2 * a32 - a22 * b3)) / det;
    const double E = (a11 * (b2 * a33 - a23 * b3)
                    - b1 * (a21 * a33 - a23 * a31)
                    + a13 * (a21 * b3 - b2 * a31)) / det;
    const double F = (a11 * (a22 * b3 - b2 * a32)
                    - a12 * (a21 * b3 - b2 * a31)
                    + b1 * (a21 * a32 - a22 * a31)) / det;
    const double cx = -D / 2.0, cy = -E / 2.0;
    const double rr = cx * cx + cy * cy - F;
    if (rr <= 1e-12) return out;
    out.c = QPointF(cx + mx, cy + my);
    out.r = std::sqrt(rr);
    out.ok = true;
    return out;
}

// 세그먼트 a..b (점 index) 가 하나의 원호로 볼 만한가
inline bool arcFits(const QList<QPointF> &pts, int a, int b, Circle &fit, double &sweepDeg,
                    bool &left)
{
    fit = fitCircle(pts, a, b);
    if (!fit.ok) return false;
    // 글자 획의 곡선은 생각보다 작다 — 높이 300mm 짜리 'D' 의 배는 반지름 40~80mm 다.
    // 예전 하한 50mm 는 그걸 통째로 걸러내서 'D' 가 MOVE 무더기로 나갔다.
    // 5m 상한은 작업장/로봇의 제한이 아니라 임의의 피팅 제한이었다. 반지름이
    // 5.00m에서 5.01m가 되는 순간 ARC 1개가 모든 MOVE로 바뀌므로 제거한다.
    if (!std::isfinite(fit.r) || fit.r < 0.02) return false;

    // 반지름 잔차 — 1mm 또는 반지름의 0.5% 중 큰 값. 이전 2%는 완만한 타원도
    // 원 하나로 흡수해 도면과 실행 경로가 달라졌다. ARC 감소보다 기하 보존이 우선이다.
    const double tol = std::max(0.001, fit.r * 0.005);
    for (int i = a; i <= b; ++i)
        if (std::abs(QLineF(fit.c, pts[i]).length() - fit.r) > tol) return false;

    // 중심각이 한 방향으로만 돌아야 한다 (S자는 두 개로 쪼갠다)
    double total = 0.0;
    int sign = 0;
    for (int i = a; i < b; ++i) {
        const QPointF u = pts[i] - fit.c, v = pts[i + 1] - fit.c;
        const double cross = u.x() * v.y() - u.y() * v.x();
        const double dot   = u.x() * v.x() + u.y() * v.y();
        const double da = std::atan2(cross, dot) * kRadToDeg;
        if (std::abs(da) < 1e-9) continue;
        // ⚠️ 한 세그먼트가 중심각을 크게 먹으면 그건 호가 아니라 **현(chord)** 이다.
        //    'D' 처럼 곧은 변의 두 끝점이 우연히 원 위에 있으면(지름이면 항상 그렇다)
        //    중간 샘플이 없어 잔차 검사를 그냥 통과해 버린다. 실측: D 자 세로변이
        //    215도짜리 가짜 ARC 로 흡수됐다. 스텝 각을 제한해 그 경로를 막는다.
        if (std::abs(da) > 30.0) return false;
        const int s = (da > 0) ? 1 : -1;
        if (sign == 0) sign = s;
        else if (s != sign) return false;
        total += da;
    }
    if (sign == 0) return false;
    sweepDeg = std::abs(total);
    // 30° 미만은 직선으로 보낸다. 세선화 잡음이 남긴 "반지름 0.67m 짜리 15° 호"
    // 같은 게 'ㅁ' 같은 직선 도형에 끼어드는 걸 막는다 (실측으로 잡은 값).
    // 360° 초과는 같은 자리를 되밟는 것 — 원호 하나일 수 없다.
    if (sweepDeg < 30.0 || sweepDeg > 360.0 + 1e-6) return false;
    left = (sign > 0);                                       // +Y 위 · CCW = 좌회전
    return true;
}

// 원 위 점에서의 접선 방위(도). left=CCW 진행 기준.
inline double tangentDeg(const Circle &c, const QPointF &p, bool left)
{
    const QPointF r = p - c.c;
    const QPointF t = left ? QPointF(-r.y(), r.x()) : QPointF(r.y(), -r.x());
    return std::atan2(t.y(), t.x()) * kRadToDeg;
}

} // namespace detail

// pts   : 펜이 지나갈 월드 좌표(미터). BLUEPRINT.points 와 같은 값.
// paint : paint[i] = pts[i-1]→pts[i] 구간을 칠하는가. paint[0] 은 미사용.
// spd   : 미리보기/예상시간 계산용 (전송하지 않음)
//
// ⚠️ 펜 오프셋 인자는 없다. 도면 그대로만 만든다 — 파일 머리 주석 참고.
// ⚠️ arcOn 인자도 없앴다 — 곡선은 **항상 ARC op** 으로 낸다. 끄는 쪽은 같은 곡선을
//    잘게 쪼갠 MOVE/TURN 으로 바꿔 동작 수만 4~10배 늘렸다(O: 4 → 49).
inline QList<Op> build(const QList<QPointF> &pts, const QList<bool> &paint,
                       const Speeds &spd = Speeds())
{
    QList<Op> out;
    const int n = pts.size();
    if (n < 2) return out;

    auto headingOf = [&](int i) {   // pts[i-1] → pts[i] 방위
        const QPointF v = pts[i] - pts[i - 1];
        return std::atan2(v.y(), v.x()) * kRadToDeg;
    };
    auto sameFlagRun = [&](int a, int b) {   // 세그먼트 a+1..b 의 paint 가 전부 같은가
        const bool f = paint.value(a + 1, true);
        for (int i = a + 2; i <= b; ++i)
            if (paint.value(i, true) != f) return false;
        return true;
    };

    double cur = headingOf(1);
    bool nozzleDown = false;

    auto setNozzle = [&](bool down, int v) {
        if (down == nozzleDown) return;
        Op o; o.kind = Op::Nozzle; o.down = down;
        o.heading = cur; o.vertex = v;
        out << o;
        nozzleDown = down;
    };
    auto turnTo = [&](double want, int v) {
        const double t = normDeg(want - cur);
        if (std::abs(t) < 0.05) { cur = want; return; }
        Op o; o.kind = Op::Turn; o.angle = t; o.speed = spd.turnDps;
        o.heading = want; o.vertex = v;
        out << o;
        cur = want;
    };

    // 🔴 노즐은 **paint 가 바뀌는 자리에서만** 움직인다.
    //    이어서 칠하는 꼭짓점(W, A 의 획 안쪽)에서는 아무것도 넣지 않는다 —
    //    "꼭짓점에서 로봇이 내부적으로 무엇을 하든 program 에는 안 드러난다"
    //    (프로토콜 §program op 규약). 올렸다 내리는 걸 끼워 넣으면 그게 곧
    //    Qt 가 꼭짓점 동작에 관여하는 것이라 규약에 어긋난다.
    //    순서는 프로토콜 §BLUEPRINT 예제 그대로: 올릴 때는 TURN **앞**,
    //    내릴 때는 TURN **뒤**. (회전 중에 도료가 새지 않게)
    auto turnAndNozzle = [&](double want, bool wantDown, int v) {
        if (!wantDown) setNozzle(false, v);
        turnTo(want, v);
        setNozzle(wantDown, v);
    };

    int i = 1;
    while (i < n) {
        // ── 원호로 묶을 수 있는지 먼저 본다 ──
        if (i + 3 < n) {
            detail::Circle best; double bestSweep = 0.0; bool bestLeft = true;
            int bestEnd = -1;
            // 최소 4세그먼트부터 시작해 최대한 길게 늘린다
            for (int b = i + 3; b < n; ++b) {
                if (!sameFlagRun(i - 1, b)) break;
                detail::Circle c; double sw; bool lf;
                if (!detail::arcFits(pts, i - 1, b, c, sw, lf)) {
                    if (bestEnd >= 0) break;      // 더 늘려도 안 되면 멈춘다
                    // 아직 후보가 없으면 좀 더 늘려 본다. 최소 스윕이 30° 라서
                    // 촘촘히 찍힌 곡선(5° 간격)은 7세그먼트는 지나야 조건을 넘는다.
                    // 예전 상한(6)이 딱 그 경계라 'ㅇ' 같은 작은 원이 통째로
                    // MOVE 무더기로 흘러나갔다 — 넉넉히 잡는다.
                    if (b > i + 48) break;
                    continue;
                }
                best = c; bestSweep = sw; bestLeft = lf; bestEnd = b;
                // 온전한 원(ㅇ·O·루프)은 **한 획 = ARC 하나(360°)** 로 보낸다.
                // 중간에서 끊으면(예전 180°/300° 캡) 나머지 몇 도가 어중간하게
                // 남아 짧은 MOVE·TURN 조각이 붙었다 — 한붓그리기가 깨진다.
                // 한 바퀴를 넘으면 같은 자리를 되밟는 것이므로 그때만 멈춘다.
                if (sw >= 360.0 - 1e-6) break;
            }
            if (bestEnd >= 0) {
                const int v = i - 1;
                const bool wantDown = paint.value(i, true);
                const double t0 = detail::tangentDeg(best, pts[v], bestLeft);
                const double t1 = detail::tangentDeg(best, pts[bestEnd], bestLeft);
                // 서버 접근 단계가 첫 op의 heading으로 이미 정렬한다. 첫 호 앞에서
                // 첫 현(chord)→접선 차이를 TURN으로 또 보내면 그 각도를 두 번 돈다.
                if (i == 1) cur = t0;
                turnAndNozzle(t0, wantDown, v);
                Op a; a.kind = Op::Arc;
                a.radius = best.r;
                a.angle = bestSweep;                       // 항상 양수
                a.arcLeft = bestLeft;
                a.dist = best.r * bestSweep * kDegToRad;   // S = R·θ
                a.paint = wantDown;
                a.speed = spd.moveFor(wantDown);
                a.heading = t0;                              // 서버 v2: ARC 진입 접선
                a.vertex = v;
                out << a;
                cur = t1;
                i = bestEnd + 1;
                continue;
            }
        }

        // ── 직진 ──
        const int v = i - 1;
        const double len = QLineF(pts[i - 1], pts[i]).length();
        if (len < 1e-6) { ++i; continue; }
        const bool wantDown = paint.value(i, true);
        turnAndNozzle(headingOf(i), wantDown, v);
        Op m; m.kind = Op::Move; m.dist = len; m.paint = wantDown;
        m.speed = spd.moveFor(wantDown);
        m.heading = cur; m.vertex = v;
        out << m;
        ++i;
    }

    // 경로는 반드시 NOZZLE up 으로 끝난다 (프로토콜 §노즐 제어의 단일 결정권)
    if (nozzleDown) {
        Op o; o.kind = Op::Nozzle; o.down = false;
        o.heading = cur; o.vertex = n - 1;
        out << o;
    }
    return out;
}

// 서버가 접근(approach) 단계에서 로봇을 세워야 할 자리 = 도면 시작점 그대로.
// 펜 오프셋 이동은 서버가 draw op을 만들 때 별도로 삽입한다.
inline QPointF approachCenter(const QList<QPointF> &pts)
{
    return pts.value(0);
}

inline double approachHeadingDeg(const QList<QPointF> &pts)
{
    if (pts.size() < 2) return 0.0;
    const QPointF v = pts[1] - pts[0];
    return std::atan2(v.y(), v.x()) * kRadToDeg;
}

// 실제 서버 접근 정렬은 program에서 처음 발견한 heading_deg를 쓴다. 특히 ARC는
// 첫 두 점의 현 방향이 아니라 정확한 진입 접선이어야 한다.
inline double approachHeadingDeg(const QList<Op> &ops, const QList<QPointF> &pts)
{
    if (!ops.isEmpty()) return ops.first().heading;
    return approachHeadingDeg(pts);
}

// 서버 v2가 BLUEPRINT를 거부할 도색 ARC를 전송 전에 찾는다. 기구 보정은 하지 않고
// 현재 서버 입력 한계만 같은 값으로 검사한다. 반환값은 op index, 없으면 -1.
inline int firstTooTightPaintArc(const QList<Op> &ops, double *radiusM = nullptr,
                                 double minRadiusM = kServerConfirmedMinPaintRadiusM)
{
    for (int i = 0; i < ops.size(); ++i) {
        const Op &o = ops[i];
        if (o.kind == Op::Arc && o.paint && o.radius + 1e-9 < minRadiusM) {
            if (radiusM) *radiusM = o.radius;
            return i;
        }
    }
    return -1;
}

// 실제 주행 거리 합 (후진 포함, 절댓값). 예상 시간 계산용.
inline double totalTravelM(const QList<Op> &ops)
{
    double s = 0.0;
    for (const Op &o : ops)
        if (o.kind == Op::Move || o.kind == Op::Arc) s += std::abs(o.dist);
    return s;
}

inline int countOf(const QList<Op> &ops, Op::Kind k)
{
    int n = 0;
    for (const Op &o : ops) if (o.kind == k) ++n;
    return n;
}

// 시퀀스 전체 소요 시간 (초). 거리·각도를 각 op 의 속도로 나눠 더한다.
// nozzleSec: 노즐을 올리고 내리는 데 걸리는 시간
inline double totalSeconds(const QList<Op> &ops, double nozzleSec = 0.5)
{
    double t = 0.0;
    for (const Op &o : ops) {
        switch (o.kind) {
        case Op::Move:
        case Op::Arc:
            if (o.speed > 1e-6) t += std::abs(o.dist) / o.speed;
            break;
        case Op::Turn:
            if (o.speed > 1e-6) t += std::abs(o.angle) / o.speed;
            break;
        case Op::Nozzle:
            t += nozzleSec;
            break;
        }
    }
    return t;
}

} // namespace motionprogram

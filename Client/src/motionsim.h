#pragma once
// ── 동작 시퀀스 재생기 ────────────────────────────────────────────────────
//
// motionprogram::build() 가 만든 시퀀스를 **실제 시간으로 그대로 실행**한다.
// 로봇 기구를 그대로 흉내낸다: 펜 = 중심 - d·(바라보는 방향).
//
// 테스트 모드 미리보기(로봇 없이 화면에서 확인)와 mp_test 가 같은 코드를 쓴다.
// ⚠️ 여기서 경로를 다시 계산하지 말 것 — 재생기가 시퀀스를 해석하는 방식이
//    로봇 펌웨어와 달라지는 순간 미리보기가 거짓말을 하게 된다. 이 파일은
//    op 를 순서대로 "틀어 주기만" 한다.
//
// 부호 규약은 motionprogram 과 동일:
//   Move.dist  음수 = 후진 (바라보는 방위는 바뀌지 않는다)
//   Turn.angle 양수 = 좌회전
#include "motionprogram.h"

namespace motionprogram {

class Player {
public:
    // ops        : 실행할 시퀀스
    // startStart : 시작 위치 = approachCenter(pts)
    // startHead  : 시작 방위(deg) = approachHeadingDeg(pts)
    //
    // ⚠️ 이 재생기가 다루는 점은 **노즐(펜)** 이다. 시퀀스가 도면 그대로(펜 기준)라
    //    노즐이 도면 위를 정확히 따라가고, 로봇은 155mm 오프셋을 자기 안에서 흡수한다.
    //    화면의 섀시(= ArUco 마커 기준점)는 여기서 155mm **앞**이며, 그 보정은
    //    Backend::pushSimPoseToView() 가 한다 — 재생기는 관여하지 않는다.
    //
    // ⚠️ 예전에는 penOffM 인자로 오프셋을 받았는데, 시퀀스가 펜 기준으로 바뀐 뒤로는
    //    호출부가 항상 0.0 만 넘겼다(넣으면 미리보기 펜이 d 만큼 뒤처져 보였다).
    //    영원히 0 인 인자라 지웠다.
    void load(const QList<Op> &ops, const QPointF &startPos, double startHeadDeg,
              double nozzleSec = 0.5)
    {
        m_ops = ops;
        m_start = startPos;
        m_startHead = startHeadDeg;
        m_nozzleSec = qMax(0.0, nozzleSec);
        m_i = 0;
        m_t = 0.0;
        m_paintTotal = 0.0;
        for (const Op &o : m_ops)
            if ((o.kind == Op::Move || o.kind == Op::Arc) && o.paint)
                m_paintTotal += std::abs(o.dist);
    }

    void clear() { m_ops.clear(); m_i = 0; m_t = 0.0; m_paintTotal = 0.0; }

    bool empty() const { return m_ops.isEmpty(); }
    bool finished() const { return m_i >= m_ops.size(); }
    // ⚠️ opIndex()/opCount() 는 지웠다 — 호출부 0. 재생 상태는 phaseText() 와
    //    progress01() 로만 화면에 나간다.

    // dtSec 만큼 시간을 흘린다. op 경계를 넘어가면 남은 시간을 다음 op 로 이어 준다
    // (한 프레임 안에서 노즐 op 여러 개를 지나가는 경우가 흔하다).
    // 반환값: 아직 실행할 op 가 남아 있는가.
    bool step(double dtSec)
    {
        double left = qMax(0.0, dtSec);
        while (left > 0.0 && m_i < m_ops.size()) {
            const double dur = durationOf(m_ops[m_i]);
            const double rest = dur - m_t;
            if (left < rest) { m_t += left; return true; }
            left -= qMax(0.0, rest);
            ++m_i;
            m_t = 0.0;
        }
        return m_i < m_ops.size();
    }

    // ── 지금 상태 ──
    // 매번 처음부터 다시 굴린다. op 는 많아야 수백 개라 비용이 없고, 프레임마다
    // 누적하지 않으니 오차가 쌓이지 않는다.
    QPointF center() const { return state().center; }
    double headingDeg() const { return state().head; }
    bool nozzleDown() const { return state().down; }
    // ⚠️ paintedM()/paintTotalM() 도 지웠다 — 호출부 0. 두 값의 비(比)인
    //    progress01() 만 쓰인다. m_paintTotal 자체는 거기서 분모로 살아 있다.

    // ⚠️ 여기 있던 pen() 은 지웠다. m_d 가 항상 0 이라 center() 와 완전히 같은 값을
    //    돌려줬다 — 두 이름이 다른 점인 것처럼 보여서 오히려 헷갈렸다.
    //    center() 가 곧 노즐 위치다(위 load() 주석 참고).

    // 도색 진행률 — 빈 이동·회전은 진행률을 올리지 않는다. 화면의 미션 오버레이가
    // "칠해진 길이"를 그대로 채우므로 두 값이 어긋나지 않는다.
    double progress01() const
    {
        if (m_paintTotal < 1e-9) return finished() ? 1.0 : 0.0;
        return qBound(0.0, state().painted / m_paintTotal, 1.0);
    }

    double durationOf(const Op &o) const
    {
        switch (o.kind) {
        case Op::Move:
        case Op::Arc:    return o.speed > 1e-6 ? std::abs(o.dist) / o.speed : 0.0;
        case Op::Turn:   return o.speed > 1e-6 ? std::abs(o.angle) / o.speed : 0.0;
        case Op::Nozzle: return m_nozzleSec;
        }
        return 0.0;
    }

    // 남은 시간(초) — 진행 중인 op 의 잔여분 + 이후 op 전부
    double remainingSec() const
    {
        double t = 0.0;
        for (int k = m_i; k < m_ops.size(); ++k)
            t += durationOf(m_ops[k]);
        return qMax(0.0, t - m_t);
    }

    // 지금 실행 중인 동작을 사람 말로 (상태 표시용)
    QString phaseText() const
    {
        if (m_i >= m_ops.size()) return QStringLiteral("완료");
        const Op &o = m_ops[m_i];
        switch (o.kind) {
        case Op::Nozzle: return o.down ? QStringLiteral("노즐 내림") : QStringLiteral("노즐 올림");
        case Op::Turn:   return QStringLiteral("제자리 회전 %1°").arg(o.angle, 0, 'f', 0);
        case Op::Arc:
            return QStringLiteral("%1 곡선 %2° (R %3 m)")
                       .arg(o.paint ? QStringLiteral("도색") : QStringLiteral("이동"))
                       .arg(o.angle, 0, 'f', 0)
                       .arg(o.radius, 0, 'f', 2);
        case Op::Move:
            if (o.paint) return QStringLiteral("도색 주행");
            if (o.dist < 0) return QStringLiteral("후진");
            return QStringLiteral("이동");
        }
        return QString();
    }

private:
    struct St {
        QPointF center;
        double head = 0.0;
        bool down = false;
        double painted = 0.0;
    };

    St state() const
    {
        St s;
        s.center = m_start;
        s.head = m_startHead;
        for (int k = 0; k <= m_i && k < m_ops.size(); ++k) {
            const Op &o = m_ops[k];
            double f = 1.0;
            if (k == m_i) {
                const double dur = durationOf(o);
                f = (dur > 1e-9) ? qBound(0.0, m_t / dur, 1.0) : 0.0;
            }
            switch (o.kind) {
            case Op::Nozzle:
                if (f >= 1.0) s.down = o.down;   // 노즐은 다 내려간 뒤에 반영
                break;
            case Op::Turn:
                s.head += o.angle * f;
                break;
            case Op::Move: {
                // 직진·후진 중에는 방위가 바뀌지 않는다
                const double r = s.head * kPi / 180.0;
                s.center += QPointF(std::cos(r), std::sin(r)) * (o.dist * f);
                if (o.paint) s.painted += std::abs(o.dist) * f;
                break;
            }
            case Op::Arc: {
                // 원호 주행 — 진행한 만큼 방위도 같이 돈다.
                // 회전중심은 진행 방향의 왼쪽(좌회전) / 오른쪽(우회전)으로 R 만큼.
                const double sweep = o.angle * f * (o.arcLeft ? 1.0 : -1.0);
                const double h0 = s.head * kPi / 180.0;
                const QPointF nrm = o.arcLeft ? QPointF(-std::sin(h0), std::cos(h0))
                                              : QPointF(std::sin(h0), -std::cos(h0));
                const QPointF c = s.center + nrm * o.radius;
                // 좌회전(CCW)이면 중심 둘레도 CCW(+) 로 돈다 — 부호가 같다
                const double a = sweep * kPi / 180.0;
                const QPointF rel = s.center - c;
                const double ca = std::cos(a), sa = std::sin(a);
                s.center = c + QPointF(rel.x() * ca - rel.y() * sa,
                                       rel.x() * sa + rel.y() * ca);
                s.head += sweep;
                if (o.paint) s.painted += std::abs(o.dist) * f;
                break;
            }
            }
        }
        return s;
    }

    QList<Op> m_ops;
    QPointF m_start;
    double m_startHead = 0.0;
    double m_nozzleSec = 0.5;
    int m_i = 0;        // 실행 중인 op
    double m_t = 0.0;   // 그 op 안에서 흐른 시간(초)
    double m_paintTotal = 0.0;
};

} // namespace motionprogram

#pragma once
// ── 스트로크(단선) 폰트 ────────────────────────────────────────────────
//
// 글자를 **획 그대로** 정의한다. 'A' 는 선 3개, 'O' 는 원 하나다.
//
// 왜 이렇게 하나:
//   예전에는 폰트를 굵게 렌더 → Zhang-Suen 세선화 → 중심선 추적으로 획을 뽑았다.
//   그런데 세선화는 획이 만나는 곳(A 의 가로대, K 의 교차점)에서 잔가지를 만들고
//   모서리를 둥글게 깎는다. 실측: 'A' 한 글자가 20점짜리 덩어리가 되고 서버로는
//   MOVE 260개 + 가짜 ARC 9개로 나갔다. 잡음을 걸러내도 원본이 이미 3획이 아니다.
//
//   CNC 각인·펜플로터가 쓰는 방법이 이거다 — 글자를 처음부터 선/호로 적어 둔다.
//   도로 표시는 예쁜 글꼴이 필요 없고, 로봇이 지나갈 자취만 정확하면 된다.
//
// 좌표계: em 박스. x 는 왼쪽 0 부터, y 는 **아래 0 · 위 1** (베이스라인 기준).
//         쓰는 쪽에서 크기·위치·y 뒤집기를 처리한다.
//
// 표기법 (문자열 한 줄이 글자 하나):
//   M x,y        새 획 시작
//   L x,y        직선
//   A cx,cy,d x,y  중심 (cx,cy) 로 도는 원호. d = 'L'(좌회전/CCW) | 'R'(우회전/CW)
//   획과 획 사이는 공백. 예) "A" = "M0,0 L0.3,1 L0.6,0 M0.12,0.36 L0.48,0.36"
#include <QChar>
#include <QList>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QVector>
#include <cmath>
#include <cstring>

namespace strokefont {

constexpr double kPi = 3.14159265358979323846;

// 글자 하나 = 획 여러 개. 획 하나 = 점 목록 (호는 이미 잘게 나눠 넣는다).
using Stroke = QVector<QPointF>;
using Glyph  = QList<Stroke>;

// ── 라틴 대문자 ───────────────────────────────────────────────────────
// 폭은 0.6 로 통일 (I 등 일부는 좁게). 높이는 0~1.
inline const char *latinDef(QChar ch)
{
    switch (ch.unicode()) {
    case 'A': return "M0,0 L0.3,1 L0.6,0 M0.12,0.4 L0.48,0.4";
    case 'B': return "M0,0 L0,1 L0.3,1 A0.3,0.75,R0.3,0.5 L0,0.5 "
                     "M0,0.5 L0.32,0.5 A0.32,0.25,R0.32,0 L0,0";
    case 'C': return "M0.58,0.78 A0.3,0.5,L0.58,0.22";
    case 'D': return "M0,0 L0,1 L0.2,1 A0.2,0.5,R0.2,0 L0,0";
    case 'E': return "M0.55,1 L0,1 L0,0 L0.55,0 M0,0.5 L0.42,0.5";
    case 'F': return "M0.55,1 L0,1 L0,0 M0,0.5 L0.42,0.5";
    case 'G': return "M0.58,0.78 A0.3,0.5,L0.58,0.22 M0.58,0.22 L0.58,0.45 L0.34,0.45";
    case 'H': return "M0,0 L0,1 M0.55,0 L0.55,1 M0,0.5 L0.55,0.5";
    case 'I': return "M0.15,0 L0.15,1";
    case 'J': return "M0.5,1 L0.5,0.25 A0.25,0.25,R0,0.25";
    case 'K': return "M0,0 L0,1 M0.55,1 L0,0.45 M0.16,0.58 L0.55,0";
    case 'L': return "M0,1 L0,0 L0.5,0";
    case 'M': return "M0,0 L0,1 L0.3,0.35 L0.6,1 L0.6,0";
    case 'N': return "M0,0 L0,1 L0.55,0 L0.55,1";
    case 'O': return "M0.3,1 A0.3,0.5,L0.3,0 A0.3,0.5,L0.3,1";
    case 'P': return "M0,0 L0,1 L0.3,1 A0.3,0.75,R0.3,0.5 L0,0.5";
    case 'Q': return "M0.3,1 A0.3,0.5,L0.3,0 A0.3,0.5,L0.3,1 M0.38,0.22 L0.6,0";
    case 'R': return "M0,0 L0,1 L0.3,1 A0.3,0.75,R0.3,0.5 L0,0.5 M0.26,0.5 L0.56,0";
    case 'S': return "M0.55,0.82 A0.28,0.75,L0.28,0.5 A0.28,0.25,R0.02,0.18";
    case 'T': return "M0,1 L0.6,1 M0.3,1 L0.3,0";
    // ⚠️ 호는 **시작점과 끝점이 같은 원 위**에 있어야 한다 (중심에서 같은 거리).
    //    어긋나면 시작점 반지름으로 돌다가 엉뚱한 데서 끝난다.
    //    U 는 아래로 돌아야 하므로 좌회전(L). 우회전이면 위로 넘어간다.
    case 'U': return "M0,1 L0,0.28 A0.28,0.28,L0.56,0.28 L0.56,1";
    case 'V': return "M0,1 L0.3,0 L0.6,1";
    case 'W': return "M0,1 L0.15,0 L0.3,0.6 L0.45,0 L0.6,1";
    case 'X': return "M0,0 L0.55,1 M0,1 L0.55,0";
    case 'Y': return "M0,1 L0.3,0.52 L0.6,1 M0.3,0.52 L0.3,0";
    case 'Z': return "M0,1 L0.55,1 L0,0 L0.55,0";
    case '0': return "M0.28,1 A0.28,0.5,L0.28,0 A0.28,0.5,L0.28,1";
    case '1': return "M0.1,0.82 L0.28,1 L0.28,0";
    case '2': return "M0.031,0.802 A0.28,0.78,R0.5215,0.7153 L0,0 L0.55,0";
    case '3': return "M0.03,0.85 A0.26,0.76,R0.26,0.52 A0.26,0.26,R0.02,0.14";
    case '4': return "M0.42,0 L0.42,1 L0,0.3 L0.56,0.3";
    case '5': return "M0.52,1 L0.08,1 L0.08,0.58 L0.28,0.58 A0.28,0.3,R0,0.3";
    case '6': return "M0.391,0.743 A0.52,0.26,L0.02,0.26 "
                     "M0.02,0.26 A0.28,0.26,L0.54,0.26 A0.28,0.26,L0.02,0.26";
    case '7': return "M0,1 L0.55,1 L0.2,0";
    case '8': return "M0.28,0.52 A0.28,0.76,L0.28,0.52 A0.28,0.26,L0.28,0.52";
    case '9': return "M0.169,0.257 A0.04,0.74,L0.54,0.74 "
                     "M0.54,0.74 A0.28,0.74,L0.02,0.74 A0.28,0.74,L0.54,0.74";
    case '.': return "M0.1,0 L0.14,0";
    case '-': return "M0.05,0.5 L0.45,0.5";
    case '/': return "M0,0 L0.5,1";
    default:  return nullptr;
    }
}

// ── 한글 자모 ─────────────────────────────────────────────────────────
// 자모마다 획을 적어 두고, 조합형 글자는 초성·중성·종성으로 쪼개 배치한다.
// 그래야 11,172자를 전부 따로 그리지 않아도 된다.
// 좌표는 자모 하나가 0~1 박스를 꽉 채운다고 보고 적는다.
inline const char *jamoDef(int idx, int kind)   // kind 0=초성 1=중성 2=종성
{
    static const char *cho[19] = {
        /*ㄱ*/ "M0,1 L1,1 L1,0",
        /*ㄲ*/ "M0,1 L0.4,1 L0.4,0 M0.6,1 L1,1 L1,0",
        /*ㄴ*/ "M0,1 L0,0 L1,0",
        /*ㄷ*/ "M0,1 L1,1 M0,1 L0,0 L1,0",
        /*ㄸ*/ "M0,1 L0.4,1 M0,1 L0,0 L0.4,0 M0.6,1 L1,1 M0.6,1 L0.6,0 L1,0",
        /*ㄹ*/ "M0,1 L1,1 L1,0.55 L0,0.55 L0,0 L1,0",
        /*ㅁ*/ "M0,0 L0,1 L1,1 L1,0 L0,0",
        /*ㅂ*/ "M0,1 L0,0 L1,0 L1,1 M0,0.5 L1,0.5",
        /*ㅃ*/ "M0,1 L0,0 L0.4,0 L0.4,1 M0,0.5 L0.4,0.5 M0.6,1 L0.6,0 L1,0 L1,1 M0.6,0.5 L1,0.5",
        /*ㅅ*/ "M0,0 L0.5,1 L1,0",
        /*ㅆ*/ "M0,0 L0.25,1 L0.5,0 M0.5,0 L0.75,1 L1,0",
        /*ㅇ*/ "M0.5,1 A0.5,0.5,L0.5,0 A0.5,0.5,L0.5,1",
        /*ㅈ*/ "M0,1 L1,1 M0.5,1 L0,0 M0.5,1 L1,0",
        /*ㅉ*/ "M0,1 L0.45,1 M0.22,1 L0,0 M0.22,1 L0.45,0 M0.55,1 L1,1 M0.78,1 L0.55,0 M0.78,1 L1,0",
        /*ㅊ*/ "M0.5,1 L0.5,0.85 M0,0.7 L1,0.7 M0.5,0.7 L0,0 M0.5,0.7 L1,0",
        /*ㅋ*/ "M0,1 L1,1 L1,0 M0,0.5 L1,0.5",
        /*ㅌ*/ "M0,1 L1,1 M0,1 L0,0 L1,0 M0,0.5 L1,0.5",
        /*ㅍ*/ "M0,1 L1,1 M0.3,1 L0.3,0 M0.7,1 L0.7,0 M0,0 L1,0",
        /*ㅎ*/ "M0.2,1 L0.8,1 M0,0.78 L1,0.78 M0.5,0.6 A0.5,0.3,L0.5,0 A0.5,0.3,L0.5,0.6",
    };
    static const char *jung[21] = {
        /*ㅏ*/ "M0.4,1 L0.4,0 M0.4,0.5 L1,0.5",
        /*ㅐ*/ "M0.25,1 L0.25,0 M0.25,0.5 L0.7,0.5 M0.85,1 L0.85,0",
        /*ㅑ*/ "M0.4,1 L0.4,0 M0.4,0.68 L1,0.68 M0.4,0.32 L1,0.32",
        /*ㅒ*/ "M0.25,1 L0.25,0 M0.25,0.68 L0.7,0.68 M0.25,0.32 L0.7,0.32 M0.85,1 L0.85,0",
        /*ㅓ*/ "M0.7,1 L0.7,0 M0.1,0.5 L0.7,0.5",
        /*ㅔ*/ "M0.55,1 L0.55,0 M0.1,0.5 L0.55,0.5 M0.85,1 L0.85,0",
        /*ㅕ*/ "M0.7,1 L0.7,0 M0.1,0.68 L0.7,0.68 M0.1,0.32 L0.7,0.32",
        /*ㅖ*/ "M0.55,1 L0.55,0 M0.1,0.68 L0.55,0.68 M0.1,0.32 L0.55,0.32 M0.85,1 L0.85,0",
        /*ㅗ*/ "M0.5,0.9 L0.5,0.35 M0,0.35 L1,0.35",
        /*ㅘ*/ "M0.3,0.9 L0.3,0.35 M0,0.35 L0.6,0.35 M0.75,1 L0.75,0 M0.75,0.6 L1,0.6",
        /*ㅙ*/ "M0.25,0.9 L0.25,0.35 M0,0.35 L0.5,0.35 M0.65,1 L0.65,0 M0.65,0.6 L0.85,0.6 M0.95,1 L0.95,0",
        /*ㅚ*/ "M0.35,0.9 L0.35,0.35 M0,0.35 L0.7,0.35 M0.85,1 L0.85,0",
        /*ㅛ*/ "M0.3,0.9 L0.3,0.35 M0.7,0.9 L0.7,0.35 M0,0.35 L1,0.35",
        /*ㅜ*/ "M0,0.65 L1,0.65 M0.5,0.65 L0.5,0.1",
        /*ㅝ*/ "M0,0.65 L0.6,0.65 M0.3,0.65 L0.3,0.1 M0.75,1 L0.75,0 M0.5,0.6 L0.75,0.6",
        /*ㅞ*/ "M0,0.65 L0.5,0.65 M0.25,0.65 L0.25,0.1 M0.65,1 L0.65,0 M0.5,0.6 L0.65,0.6 M0.95,1 L0.95,0",
        /*ㅟ*/ "M0,0.65 L0.7,0.65 M0.35,0.65 L0.35,0.1 M0.85,1 L0.85,0",
        /*ㅠ*/ "M0,0.65 L1,0.65 M0.3,0.65 L0.3,0.1 M0.7,0.65 L0.7,0.1",
        /*ㅡ*/ "M0,0.5 L1,0.5",
        /*ㅢ*/ "M0,0.5 L0.7,0.5 M0.85,1 L0.85,0",
        /*ㅣ*/ "M0.5,1 L0.5,0",
    };
    static const char *jong[28] = {
        "",                                  /* 없음 */
        /*ㄱ*/ "M0,1 L1,1 L1,0",
        /*ㄲ*/ "M0,1 L0.4,1 L0.4,0 M0.6,1 L1,1 L1,0",
        /*ㄳ*/ "M0,1 L0.45,1 L0.45,0 M0.55,0 L0.78,1 L1,0",
        /*ㄴ*/ "M0,1 L0,0 L1,0",
        /*ㄵ*/ "M0,1 L0,0 L0.45,0 M0.55,1 L1,1 M0.78,1 L0.55,0 M0.78,1 L1,0",
        /*ㄶ*/ "M0,1 L0,0 L0.45,0 M0.62,1 L0.9,1 M0.55,0.78 L1,0.78 M0.78,0.6 A0.78,0.3,L0.78,0 A0.78,0.3,L0.78,0.6",
        /*ㄷ*/ "M0,1 L1,1 M0,1 L0,0 L1,0",
        /*ㄹ*/ "M0,1 L1,1 L1,0.55 L0,0.55 L0,0 L1,0",
        /*ㄺ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,1 L1,1 L1,0",
        /*ㄻ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,0 L0.55,1 L1,1 L1,0 L0.55,0",
        /*ㄼ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,1 L0.55,0 L1,0 L1,1 M0.55,0.5 L1,0.5",
        /*ㄽ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,0 L0.78,1 L1,0",
        /*ㄾ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,1 L1,1 M0.55,1 L0.55,0 L1,0 M0.55,0.5 L1,0.5",
        /*ㄿ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.55,1 L1,1 M0.68,1 L0.68,0 M0.87,1 L0.87,0 M0.55,0 L1,0",
        /*ㅀ*/ "M0,1 L0.45,1 L0.45,0.55 L0,0.55 L0,0 L0.45,0 M0.62,1 L0.9,1 M0.55,0.78 L1,0.78 M0.78,0.6 A0.78,0.3,L0.78,0 A0.78,0.3,L0.78,0.6",
        /*ㅁ*/ "M0,0 L0,1 L1,1 L1,0 L0,0",
        /*ㅂ*/ "M0,1 L0,0 L1,0 L1,1 M0,0.5 L1,0.5",
        /*ㅄ*/ "M0,1 L0,0 L0.45,0 L0.45,1 M0,0.5 L0.45,0.5 M0.55,0 L0.78,1 L1,0",
        /*ㅅ*/ "M0,0 L0.5,1 L1,0",
        /*ㅆ*/ "M0,0 L0.25,1 L0.5,0 M0.5,0 L0.75,1 L1,0",
        /*ㅇ*/ "M0.5,1 A0.5,0.5,L0.5,0 A0.5,0.5,L0.5,1",
        /*ㅈ*/ "M0,1 L1,1 M0.5,1 L0,0 M0.5,1 L1,0",
        /*ㅊ*/ "M0.5,1 L0.5,0.85 M0,0.7 L1,0.7 M0.5,0.7 L0,0 M0.5,0.7 L1,0",
        /*ㅋ*/ "M0,1 L1,1 L1,0 M0,0.5 L1,0.5",
        /*ㅌ*/ "M0,1 L1,1 M0,1 L0,0 L1,0 M0,0.5 L1,0.5",
        /*ㅍ*/ "M0,1 L1,1 M0.3,1 L0.3,0 M0.7,1 L0.7,0 M0,0 L1,0",
        /*ㅎ*/ "M0.2,1 L0.8,1 M0,0.78 L1,0.78 M0.5,0.6 A0.5,0.3,L0.5,0 A0.5,0.3,L0.5,0.6",
    };
    if (kind == 0) return (idx >= 0 && idx < 19) ? cho[idx] : nullptr;
    if (kind == 1) return (idx >= 0 && idx < 21) ? jung[idx] : nullptr;
    return (idx > 0 && idx < 28) ? jong[idx] : nullptr;
}

// ── 파서 ──────────────────────────────────────────────────────────────
// 호는 여기서 10° 간격으로 잘게 나눠 넣는다. 정확한 원 위의 점이므로
// motionprogram 의 원호 검출이 반지름 오차 없이 ARC 로 되돌린다.
// arcStepDeg: 호를 몇 도 간격으로 점 찍을지.
//   너무 촘촘하면(5°) ARC 를 끄고 폴리라인으로 보낼 때 op 이 폭발한다
//   (실측: 'STOP' 이 437동작). 15° 면 반지름 150mm 기준 현 길이 39mm 로
//   붓 폭(50mm)보다 작아 눈에 각이 안 보이고, ARC 검출도 그대로 걸린다.
inline Glyph parse(const QString &def, double arcStepDeg = 15.0)
{
    Glyph out;
    Stroke cur;
    QPointF at;
    const QStringList toks = def.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    for (const QString &t : toks) {
        if (t.size() < 2) continue;
        const QChar op = t.at(0);
        const QString arg = t.mid(1);
        const QStringList f = arg.split(QLatin1Char(','));
        if (op == QLatin1Char('M')) {
            if (cur.size() >= 2) out.append(cur);
            cur.clear();
            if (f.size() < 2) continue;
            at = QPointF(f[0].toDouble(), f[1].toDouble());
            cur.append(at);
        } else if (op == QLatin1Char('L')) {
            if (f.size() < 2) continue;
            at = QPointF(f[0].toDouble(), f[1].toDouble());
            cur.append(at);
        } else if (op == QLatin1Char('A')) {
            // A cx,cy,Dx,y — 세 번째 항목이 "L0.58" 처럼 **방향 문자로 시작**한다.
            // ⚠️ 여기서 앞뒤를 헷갈리면 호가 전부 x=0 으로 가고 방향도 한쪽으로
            //    고정된다 ('C' 가 갈고리로 나왔던 원인).
            if (f.size() < 4) continue;
            const QPointF c(f[0].toDouble(), f[1].toDouble());
            QString dirTok = f[2];
            bool ccw = true;
            if (!dirTok.isEmpty() && (dirTok.at(0) == QLatin1Char('L')
                                   || dirTok.at(0) == QLatin1Char('R'))) {
                ccw = (dirTok.at(0) == QLatin1Char('L'));
                dirTok = dirTok.mid(1);
            }
            const QPointF to(dirTok.toDouble(), f[3].toDouble());
            const double r = std::hypot(at.x() - c.x(), at.y() - c.y());
            double a0 = std::atan2(at.y() - c.y(), at.x() - c.x());
            double a1 = std::atan2(to.y() - c.y(), to.x() - c.x());
            double sweep = a1 - a0;
            if (ccw)  { while (sweep <= 0) sweep += 2 * kPi; }
            else      { while (sweep >= 0) sweep -= 2 * kPi; }
            const int n = std::max(2, int(std::ceil(std::abs(sweep) * 180.0 / kPi / arcStepDeg)));
            for (int i = 1; i <= n; ++i) {
                const double a = a0 + sweep * (double(i) / n);
                cur.append(QPointF(c.x() + std::cos(a) * r, c.y() + std::sin(a) * r));
            }
            at = to;
        }
    }
    if (cur.size() >= 2) out.append(cur);
    return out;
}

// 글자 하나의 획들. 없으면 빈 목록.
// advance 에 글자 폭(em 기준)을 돌려준다.
inline Glyph glyphFor(QChar ch, double *advance)
{
    const ushort u = ch.unicode();

    // 한글 완성형 — 초성·중성·종성으로 쪼개 배치
    if (u >= 0xAC00 && u <= 0xD7A3) {
        const int s = u - 0xAC00;
        const int L = s / (21 * 28), V = (s % (21 * 28)) / 28, T = s % 28;
        // 중성이 세로형(ㅏㅑㅓㅕㅣ...)인지 가로형(ㅗㅛㅜㅠㅡ)인지로 배치가 갈린다
        static const bool vertical[21] = {
            true, true, true, true, true, true, true, true,      // ㅏㅐㅑㅒㅓㅔㅕㅖ
            false, false, false, false, false, false, false, false, false, false, // ㅗ~ㅢ 앞부분
            false, false, true };                                 // ㅡ ㅢ ㅣ
        // ⚠️ 원이 든 자모(ㅇ·ㅎ)는 **가로세로 같은 배율**로 박스 안에 맞춘다.
        //    가로만 눌러 넣으면 원이 타원이 되고, 타원은 ARC(원호) 하나로 보낼 수
        //    없어 로봇 시퀀스가 짧은 직선 무더기로 깨진다 — 한붓그리기가 안 된다.
        auto placeDef = [](const char *def, double ox, double oy, double sx, double sy) {
            Glyph o;
            if (!def || !*def) return o;
            if (std::strchr(def, 'A')) {
                const double u = std::min(sx, sy);
                ox += (sx - u) * 0.5;
                oy += (sy - u) * 0.5;
                sx = sy = u;
            }
            for (const Stroke &st : parse(QString::fromUtf8(def))) {
                Stroke n;
                for (const QPointF &p : st)
                    n.append(QPointF(ox + p.x() * sx, oy + p.y() * sy));
                o.append(n);
            }
            return o;
        };
        Glyph out;
        const double top = (T > 0) ? 0.34 : 0.0;      // 받침이 있으면 위로 올린다
        const double hgt = (T > 0) ? 0.66 : 1.0;
        if (vertical[V]) {
            out += placeDef(jamoDef(L, 0), 0.02, top, 0.52, hgt);
            out += placeDef(jamoDef(V, 1), 0.56, top, 0.42, hgt);
        } else {
            out += placeDef(jamoDef(L, 0), 0.20, top + hgt * 0.42, 0.60, hgt * 0.58);
            out += placeDef(jamoDef(V, 1), 0.02, top, 0.96, hgt * 0.42);
        }
        if (T > 0)
            out += placeDef(jamoDef(T, 2), 0.20, 0.0, 0.60, 0.32);
        if (advance) *advance = 1.05;
        return out;
    }

    // 낱자로 쓴 한글 자모 (호환 자모 U+3131~U+3163)
    if (u >= 0x3131 && u <= 0x3163) {
        if (advance) *advance = 0.75;
        if (u >= 0x314F) {                       // 모음 ㅏ~ㅣ → 중성 0..20
            const char *d = jamoDef(u - 0x314F, 1);
            return d ? parse(QString::fromUtf8(d)) : Glyph();
        }
        // 자음 30개 → 종성표 index (ㄸㅃㅉ 는 종성에 없어 초성표를 쓴다)
        static const int toJong[30] = { 1,2,3,4,5,6,7,-1,8,9,10,11,12,13,14,15,
                                        16,17,-1,18,19,20,21,22,-1,23,24,25,26,27 };
        static const int toCho[30]  = { 0,1,-1,2,-1,-1,3,4,5,-1,-1,-1,-1,-1,-1,-1,
                                        6,7,8,-1,9,10,11,12,13,14,15,16,17,18 };
        const int i = u - 0x3131;
        const int j = toJong[i];
        const char *d = (j > 0) ? jamoDef(j, 2)
                                : ((toCho[i] >= 0) ? jamoDef(toCho[i], 0) : nullptr);
        return (d && *d) ? parse(QString::fromUtf8(d)) : Glyph();
    }

    // 라틴 — 소문자는 대문자로 (도로 표시는 대문자를 쓴다)
    const QChar up = ch.toUpper();
    if (const char *d = latinDef(up)) {
        if (advance) *advance = (up == QLatin1Char('I')) ? 0.32 : 0.72;
        return parse(QString::fromUtf8(d));
    }
    if (ch == QLatin1Char(' ')) { if (advance) *advance = 0.45; return {}; }
    if (advance) *advance = 0.0;
    return {};
}

// 문자열 전체 → 획 목록. 반환 좌표는 em 기준 (y 는 위가 1).
// outWidth 에 전체 폭을 돌려준다.
inline QList<Stroke> layout(const QString &text, double *outWidth)
{
    QList<Stroke> all;
    double x = 0.0;
    for (const QChar &ch : text) {
        double adv = 0.0;
        const Glyph g = glyphFor(ch, &adv);
        for (const Stroke &st : g) {
            Stroke n;
            for (const QPointF &p : st) n.append(QPointF(p.x() + x, p.y()));
            all.append(n);
        }
        x += adv + 0.12;                 // 글자 간격
    }
    if (outWidth) *outWidth = std::max(0.001, x - 0.12);
    return all;
}

} // namespace strokefont

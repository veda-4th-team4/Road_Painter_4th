#pragma once
// ===== Qt 도면/동작 시퀀스 -> 로봇 op 시퀀스 (프로토콜 v2) =====
// 규격: docs/PROTOCOL_v2_ROBOT.md (이 파일은 그 문서의 §4·§5 구현체다)
//
// 이 파일이 책임지는 것 세 가지:
//   1. 부호 반전  - 서버 내부는 CCW 양수, 로봇 대면은 "양수 = 오른쪽" (§1.3)
//   2. 펜 오프셋 보정 op 삽입 - Qt NOZZLE은 버리고 서버가 다시 만든다 (§5.3)
//   3. arc 반지름 치환 - R_robot = sqrt(R_paint² - d²) (§5.4)
//
// 로봇에 나가는 op에는 서버 판정용 정보(목표 좌표 등)를 싣지 않는다. 대신
// 같은 길이의 OpMeta 배열을 나란히 만들어 서버가 들고 있는다 - 로봇이 읽지도
// 않을 필드로 전선을 채우지 않기 위함이고, 로그를 봤을 때 "로봇이 실제로 받은
// 것"과 "서버가 판정에 쓰는 것"이 섞이지 않는다.
#include "params.hpp"
#include "path_planner.hpp"
#include "protocol.hpp"
#include <cmath>
#include <string>
#include <vector>

// "각도 없음" 표식. Qt가 heading_deg를 빼먹은 op은 ALIGN/DRIFT/MORE 판정에서
// 조용히 제외된다 (로봇을 세워두지는 않는다 - 판정 불가는 곧 GO다).
inline constexpr double kNoHeading = 1e9;
inline bool hasHeading(double h) { return h < 1e8 && h > -1e8; }

// 🔴 서버 내부 계산은 전부 CCW 양수다. 로봇에 나가는 각도만 이 함수를 통과한다.
// 여기 말고 다른 곳에서 부호를 뒤집으면 이중 반전이 되어 조용히 깨진다 (§1.3).
inline double toRobotDeg(double ccwDeg) { return -ccwDeg; }

inline double round1(double v) { return std::round(v * 10) / 10; }
inline double round3(double v) { return std::round(v * 1000) / 1000; }

// op 하나에 대해 "서버만 아는 것". activeOps_와 index가 1:1로 대응한다.
struct OpMeta {
    std::string op;        // "move" / "turn" / "nozzle" / "arc"
    bool isPath = false;   // role == "path" (도면 동작). false면 서버가 끼운 보정
    // 이 op을 실행하는 동안 로봇이 바라보는 절대 방위 (CCW). DRIFT/ALIGN 기준.
    double headingDeg = kNoHeading;
    // 이 op을 마쳤을 때 바라보는 절대 방위 (CCW). turn/arc는 진입각과 다르다.
    double exitHeadingDeg = kNoHeading;
    bool hasTarget = false;   // MORE 판정이 가능한가 (도착 꼭짓점을 아는가)
    Pt penTarget{0, 0};       // 도착 꼭짓점 = 펜이 있어야 할 자리 (도면 좌표)
    bool painting = false;    // 이 op을 실행하는 동안 노즐이 내려가 있는가.
                              // true면 마커 중심은 꼭짓점보다 a 앞에 있어야 한다 (§5.2)
};

struct PlannedPath {
    json ops = json::array();
    std::vector<OpMeta> meta;
    bool ok = true;
    std::string failReason;  // Qt DRAW_FAIL.reason
    std::string failMsg;     // Qt DRAW_FAIL.msg
};

// op 배열과 meta 배열을 항상 같이 늘리는 작은 도우미. op_index/role 부여도
// 여기 한 곳에서만 한다 - 두 배열의 길이가 어긋날 여지를 없앤다.
class OpSeq {
public:
    void moveOp(double distM, bool isPath, double headCcw, bool hasTgt = false,
                Pt tgt = {0, 0}, bool painting = false) {
        OpMeta m;
        m.op = "move";
        m.isPath = isPath;
        m.headingDeg = m.exitHeadingDeg = headCcw;  // 직진은 방향이 안 바뀐다
        m.hasTarget = hasTgt;
        m.penTarget = tgt;
        m.painting = painting;
        push(json{{"op", "move"}, {"dist_m", round3(distM)}}, m);
    }
    // angCcw = CCW 양수 (서버 내부 규약). 전선에는 부호를 뒤집어 내보낸다.
    void turnOp(double angCcw, bool isPath, double headAfterCcw) {
        OpMeta m;
        m.op = "turn";
        m.isPath = isPath;
        m.headingDeg = m.exitHeadingDeg = headAfterCcw;
        push(json{{"op", "turn"}, {"angle_deg", round1(toRobotDeg(angCcw))}}, m);
    }
    void nozzleOp(bool down) {
        OpMeta m;
        m.op = "nozzle";
        m.isPath = false;  // 노즐 op은 전부 서버가 만든 것이다 (§5.3)
        push(json{{"op", "nozzle"}, {"down", down}}, m);
    }
    // radiusRobotM = 로봇 마커 중심이 그려야 할 반지름 (보정 완료값).
    // angleMagDeg = 회전량 크기(항상 양수), dir = "left"/"right".
    void arcOp(double radiusRobotM, double angleMagDeg, const std::string& dir,
               double radiusDrawM, double entryHeadCcw, double exitHeadCcw,
               bool hasTgt, Pt tgt, bool painting) {
        OpMeta m;
        m.op = "arc";
        m.isPath = true;
        m.headingDeg = entryHeadCcw;
        m.exitHeadingDeg = exitHeadCcw;
        m.hasTarget = hasTgt;
        m.penTarget = tgt;
        m.painting = painting;
        push(json{{"op", "arc"},
                  {"radius_m", round3(radiusRobotM)},
                  {"angle_deg", round1(angleMagDeg)},
                  {"direction", dir},
                  {"radius_draw_m", round3(radiusDrawM)}},
             m);
    }

    size_t size() const { return out_.meta.size(); }
    PlannedPath take() { return std::move(out_); }

private:
    void push(json op, OpMeta m) {
        op["op_index"] = (int)out_.ops.size();
        // role은 관측용 메타데이터다. 로봇은 읽지 않는다 (§4 상단).
        op["role"] = m.isPath ? "path" : "offset";
        out_.ops.push_back(std::move(op));
        out_.meta.push_back(std::move(m));
    }
    PlannedPath out_;
};

// Qt program 검증. BLUEPRINT 수신 즉시 불러 도면 자체를 거부한다.
// reason/msg는 Qt DRAW_FAIL에 그대로 실린다.
inline bool validateProgram(const json& program, std::string& reason,
                            std::string& msg) {
    for (const auto& q : program) {
        if (!q.is_object() || !q.contains("op")) {
            reason = "bad_program";
            msg = "op 필드 없음";
            return false;
        }
        std::string o = q.value("op", "");
        if (o != "MOVE" && o != "TURN" && o != "NOZZLE" && o != "ARC") {
            reason = "bad_program";
            msg = "지원하지 않는 op (MOVE/TURN/NOZZLE/ARC만 가능): " + o;
            return false;
        }
        if (!q.contains("v") || !q["v"].is_number_integer()) {
            reason = "bad_program";
            msg = "v(출발 꼭짓점 index) 없음";
            return false;
        }
        // 🔴 노즐이 그릴 수 있는 최소 원의 반지름은 펜 오프셋 a다 (§5.5).
        //   그보다 빡빡한 원은 어떤 바퀴 속도 조합으로도 그릴 수 없으므로
        //   NaN을 로봇에 흘려보내지 말고 여기서 도면을 거부한다.
        if (o == "ARC" && q.value("paint", false)) {
            double r = q.value("radius_m", 0.0);
            if (r < params().min_paint_radius_m) {
                reason = "arc_too_tight";
                char b[128];
                snprintf(b, sizeof b,
                         "곡선 반지름 %.3fm - 최소 도색 반지름 %.3fm 미만", r,
                         params().min_paint_radius_m);
                msg = b;
                return false;
            }
        }
    }
    return true;
}

// ===== 도색(draw) 경로 생성 =====
//
// 전제: 로봇 마커 중심이 points[0] 위에, 첫 도색 방향을 바라보고 서 있다
//       (1단계 접근이 그렇게 세워준다). 노즐은 올라가 있다.
//
// 알고리즘은 §5.3 그대로다. 불변식 하나로 요약된다:
//   노즐 down ⟺ 마커 중심이 꼭짓점보다 진행방향으로 정확히 a 앞
//   노즐 up   ⟺ 마커 중심이 꼭짓점 위
// 보정 op은 이 두 상태를 오갈 때만 삽입된다. 그래서 도색 move가 연달아 나와도
// 사이에 불필요한 노즐 up/down이 끼지 않는다.
inline PlannedPath buildDrawOps(const json& program,
                                const std::vector<Pt>& points) {
    const double a = params().pen_offset_m;
    OpSeq seq;
    bool penDown = false;
    // 노즐을 올릴 때 뒤로 물러날 방향 = 직전 도색 op의 방위 (그 방향 그대로
    // 후진한다). 로봇은 회전하지 않았으므로 여전히 그쪽을 바라보고 있다.
    double lastPaintHeading = kNoHeading;

    for (size_t i = 0; i < program.size(); ++i) {
        const json& q = program[i];
        std::string o = q.value("op", "");
        if (o == "NOZZLE") continue;  // 🔴 Qt NOZZLE은 전부 버린다 (§5.3)

        const bool isTravel = (o == "MOVE" || o == "ARC");
        const bool isPaint = isTravel && q.value("paint", false);
        const double head = q.value("heading_deg", kNoHeading);  // CCW 절대 방위

        // 도착 꼭짓점 = 그다음 Qt op의 v. 없으면 폴리라인 마지막 점 (§6.5).
        // 추측항법으로 누적하지 않고 Qt가 준 좌표만 쓴다 - 오차가 쌓이지 않는다.
        Pt tgt{0, 0};
        bool hasTgt = false;
        if (isTravel) {
            int vn = -1;
            for (size_t j = i + 1; j < program.size(); ++j) {
                if (program[j].contains("v") && program[j]["v"].is_number_integer()) {
                    vn = program[j]["v"].get<int>();
                    break;
                }
            }
            if (vn < 0) vn = (int)points.size() - 1;
            if (vn >= 0 && vn < (int)points.size()) {
                tgt = points[(size_t)vn];
                hasTgt = true;
            }
        }

        if (isPaint && !penDown) {  // 도색 진입: a 전진 후 노즐 down
            // 노즐을 올리는 op은 넣지 않는다 - penDown==false라 이미 올라가 있다.
            seq.moveOp(+a, false, head);
            seq.nozzleOp(true);
            penDown = true;
        } else if (!isPaint && penDown) {  // 도색 이탈: 노즐 up 후 a 후진
            seq.nozzleOp(false);
            seq.moveOp(-a, false, lastPaintHeading);
            penDown = false;
        }

        if (o == "MOVE") {
            seq.moveOp(q.value("dist_m", 0.0), true, head, hasTgt, tgt, isPaint);
        } else if (o == "TURN") {
            seq.turnOp(q.value("angle_deg", 0.0), true, head);
        } else if (o == "ARC") {
            const double rPaint = q.value("radius_m", 0.0);
            const double angMag = std::fabs(q.value("angle_deg", 0.0));
            std::string dir = q.value("direction", "left");
            if (dir != "left" && dir != "right") dir = "left";
            // 🔴 R_robot = sqrt(R_paint² - d²) (§5.4). ICR은 반드시 좌우 바퀴
            //   축선 위에 있고 그 축선은 마커 중심 C를 지나므로, 직각의 꼭짓점은
            //   N(노즐)이 아니라 C다. 검산: R_robot=0(제자리 회전)이면
            //   R_paint = d = 0.155m - 노즐만 반지름 d의 원을 그리는 자명한 사실과 맞다.
            // 도색하지 않는 arc는 펜이 올라가 있어 중심이 곧 도면 자취이므로
            // 보정하지 않는다 (§5.2 불변식의 나머지 절반).
            double rRobot = rPaint;
            if (isPaint) {
                // 정상 흐름에서는 validateProgram이 이미 걸렀다. 여기서 한 번 더
                // 보는 것은 sqrt 안이 음수가 되어 NaN이 로봇까지 흘러가는 경로를
                // 원천 차단하기 위한 것 (§4.4 마이그레이션 체크리스트 #3).
                if (rPaint < params().min_paint_radius_m) {
                    PlannedPath bad;
                    bad.ok = false;
                    bad.failReason = "arc_too_tight";
                    char b[128];
                    snprintf(b, sizeof b,
                             "곡선 반지름 %.3fm - 최소 도색 반지름 %.3fm 미만",
                             rPaint, params().min_paint_radius_m);
                    bad.failMsg = b;
                    return bad;
                }
                double inner = rPaint * rPaint - a * a;
                rRobot = inner > 0 ? std::sqrt(inner) : 0.0;
            }
            // left = CCW(+), right = CW(-)
            double exitHead = hasHeading(head)
                                  ? normDeg(head + (dir == "left" ? angMag : -angMag))
                                  : kNoHeading;
            seq.arcOp(rRobot, angMag, dir, rPaint, head, exitHead, hasTgt, tgt,
                      isPaint);
        }
        if (isPaint && hasHeading(head)) lastPaintHeading = head;
    }

    if (penDown) {  // 경로 끝 - 노즐은 반드시 올려두고 중심을 꼭짓점에 되돌린다
        seq.nozzleOp(false);
        seq.moveOp(-a, false, lastPaintHeading);
    }
    return seq.take();
}

// ===== 접근(approach) 경로 생성 =====
//
// 로봇 현재 pose -> 도면 시작점 -> 첫 도색 방향으로 회전. 전부 role="path"라
// ALIGN/MORE/DRIFT가 그대로 적용된다. 접근에서 빠지는 것은 오프셋 보정 op뿐이다
// (§8) - 노즐이 올라가 있으므로 MORE의 목표도 시작점 그대로다.
inline PlannedPath buildApproachOps(const Pose& start, const Pt& target,
                                    double firstHeadingDeg) {
    const Params& P = params();
    OpSeq seq;
    double dx = target[0] - start.x, dy = target[1] - start.y;
    double dist = std::hypot(dx, dy);
    double th = start.theta * 180.0 / M_PI;

    if (dist >= P.min_move_m) {
        double desired = round1(std::atan2(dy, dx) * 180.0 / M_PI);
        double turn = normDeg(desired - th);
        if (std::fabs(turn) > P.min_turn_deg) seq.turnOp(turn, true, desired);
        seq.moveOp(dist, true, desired, /*hasTgt=*/true, target,
                   /*painting=*/false);
        th = desired;
    }
    if (hasHeading(firstHeadingDeg)) {
        double turn = normDeg(firstHeadingDeg - th);
        if (std::fabs(turn) > P.min_turn_deg)
            seq.turnOp(turn, true, round1(firstHeadingDeg));
    }
    return seq.take();
}

// ===== 하위호환: Qt가 program 없이 points만 보낸 경우 =====
// 도면에서 Qt 포맷(대문자·CCW 양수)의 동작 시퀀스를 만든다. 그러면 도색 경로
// 생성은 program이 있을 때와 완전히 같은 경로(buildDrawOps)를 탄다 - 오프셋
// 보정·노즐 타이밍 규칙이 한 곳에만 존재하게 된다.
//
// 시작 방위는 "첫 구간 방향"으로 잡는다. 접근 경로 마지막 turn이 로봇을 그
// 방향으로 세워두기 때문 (그래서 첫 TURN은 자연히 생략된다).
inline json programFromPoints(const std::vector<Pt>& pts,
                              const std::vector<char>& paint) {
    const Params& P = params();
    json prog = json::array();
    if (pts.size() < 2) return prog;
    double th = std::atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]) *
                180.0 / M_PI;
    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i][0] - pts[i - 1][0], dy = pts[i][1] - pts[i - 1][1];
        double dist = std::hypot(dx, dy);
        if (dist < P.min_move_m) continue;
        double head = round1(std::atan2(dy, dx) * 180.0 / M_PI);
        double turn = normDeg(head - th);
        if (std::fabs(turn) > P.min_turn_deg)
            prog.push_back({{"op", "TURN"},
                            {"angle_deg", round1(turn)},
                            {"heading_deg", head},
                            {"v", (int)(i - 1)}});
        // paint[i] = "pts[i-1] -> pts[i] 구간을 칠하는가" (v1 BLUEPRINT 규약).
        // 길이가 어긋나면 전 구간 도색으로 본다.
        bool p = (paint.size() == pts.size()) ? paint[i] != 0 : true;
        prog.push_back({{"op", "MOVE"},
                        {"dist_m", round3(dist)},
                        {"paint", p},
                        {"heading_deg", head},
                        {"v", (int)(i - 1)}});
        th = head;
    }
    return prog;
}

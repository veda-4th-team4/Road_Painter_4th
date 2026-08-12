#pragma once
// ===== Qt 도면/동작 시퀀스 -> 로봇 op 시퀀스 (프로토콜 v2) =====
// 규격: docs/PROTOCOL_v2_ROBOT.md (이 파일은 그 문서의 §4·§5 구현체다)
//
// 이 파일이 책임지는 것 네 가지:
//   1. 부호 반전  - 서버 내부는 CCW 양수, 로봇 대면은 "양수 = 오른쪽" (§1.3)
//   2. 펜 오프셋 보정 op 삽입 - Qt NOZZLE은 버리고 서버가 다시 만든다 (§5.3)
//   3. arc 반지름 치환 - R_robot = sqrt(R_paint² - d²) (§5.4)
//   4. arc 진입 위상 보정 - 차체를 접선에서 φ만큼 틀어 세운다 (§11-2, 아래 arcPhaseDeg)
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

// ===== 도색 호의 진입 위상 φ (§11-2) =====
//
// 🔴 노즐을 내린 채 호를 그리려면 차체가 접선을 향하면 안 된다.
//
// 펜 P를 도면 원 위에 놓는 것만으로는 부족하다. 회전 중심(ICR)은 반드시 좌우
// 바퀴 축선 위에 있고 그 축선은 마커 중심 C를 지나 진행방향에 수직인데, 도면
// 원의 중심 O에 ICR을 놓으려면 C가 O에서 R_robot만큼, P에서 a만큼 떨어진
// 자리여야 한다. R_robot² + a² = R_paint² 이므로 이 삼각형은 항상 성립하고,
// 그때 차체 방위는 접선에서 정확히 φ = atan(a / R_robot)만큼 호 안쪽으로
// 틀어진 방향이다.
//
// 이걸 빼먹으면 반지름은 맞는데 **원이 통째로 다른 자리에 그려진다** -
// 드라이런 실측에서 R=0.5m 반원의 펜 도착점이 0.31m 어긋났다 (φ = 18.06도).
//
// 검산: R_paint = a (= 최소 도색 반지름, 제자리 회전)이면 R_robot = 0이라
//   φ = 90도. 차체가 접선과 직각 = 마커 중심이 원 중심에 가만히 있고 펜만
//   반지름 a로 도는 자명한 사실과 맞는다. atan2를 쓰므로 R_robot = 0에서도
//   나눗셈이 터지지 않는다.
//
// 반환값은 CCW 부호를 포함한다 (좌회전 호면 +, 우회전 호면 -).
inline double arcPhaseDeg(double rRobotM, double aM, bool left) {
    const double phi = std::atan2(aM, rRobotM) * 180.0 / M_PI;
    return left ? phi : -phi;
}

// 주행 op 하나의 기하. Qt op을 읽어 "차체가 어느 방위로 들어가고 나오는가"를
// 확정한다. 직선은 접선 = 차체 방위지만, 도색 호는 위 φ만큼 어긋나 있다.
struct TravelGeom {
    double bodyEntry = kNoHeading;  // 이 op을 시작할 때 차체가 바라보는 방위
    double bodyExit = kNoHeading;   // 마쳤을 때 바라보는 방위
    double phase = 0.0;             // 접선 대비 차체 위상 (도색 호만 0이 아니다)
    double rRobot = 0.0;            // 로봇에 실어 보낼 반지름 (arc 전용)
};

// 🔴 ARC의 heading_deg는 "호에 **진입할 때**의 접선"이다 (2026-08-07 Qt팀과 확정).
//   MOVE/TURN은 진입과 출구가 같아 구분이 필요 없지만, 호는 스윕만큼 다르므로
//   출구 접선을 여기서 더해 만든다.
//
//   경위: 저장소에 올라와 있는 Qt 코드(7/30, motionprogram.h `a.heading = t1`)는
//   출구 접선을 보내고 있어서, 그 코드에 맞추려고 한때 서버를 출구 기준으로
//   바꿨다(91ec2e2). 그 뒤 Qt팀이 "진입 접선으로 이미 수정·검증했다"고 회신해
//   되돌린 것이다. 접근 단계가 정렬시켜야 할 방위가 곧 진입 접선이므로 계약상
//   이쪽이 자연스럽다.
//
//   ⚠️ Qt의 해당 수정이 저장소에 push되기 전까지는 저장소 코드(출구)와 이
//   구현(진입)이 어긋난다. 부분호에서 스윕만큼 틀어지므로, 호 도색 통합 시험은
//   Qt push 확인 후에 잡을 것.
inline TravelGeom travelGeom(const json& q, bool isPaint, double aM) {
    TravelGeom g;
    const double head = q.value("heading_deg", kNoHeading);
    if (q.value("op", "") != "ARC") {  // 직진: 방향이 안 바뀐다
        g.bodyEntry = g.bodyExit = head;
        return g;
    }
    const double rPaint = q.value("radius_m", 0.0);
    const double sweep = std::fabs(q.value("angle_deg", 0.0));
    const bool left = (q.value("direction", "left") != "right");
    // 도색하지 않는 호는 펜이 올라가 있어 마커 중심이 곧 도면 자취다 - 반지름도
    // 위상도 보정하지 않는다 (§5.2 불변식의 나머지 절반).
    if (isPaint) {
        const double inner = rPaint * rPaint - aM * aM;
        g.rRobot = inner > 0 ? std::sqrt(inner) : 0.0;
        g.phase = arcPhaseDeg(g.rRobot, aM, left);
    } else {
        g.rRobot = rPaint;
    }
    if (!hasHeading(head)) return g;
    const double tanEntry = head;
    const double tanExit = normDeg(head + (left ? sweep : -sweep));
    g.bodyEntry = normDeg(tanEntry + g.phase);
    g.bodyExit = normDeg(tanExit + g.phase);
    return g;
}

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
    // 이 op을 마쳤을 때 마커 중심이 꼭짓점보다 진행방향으로 a 앞에 있어야 하는가.
    // 즉 §5.2 불변식 중 "노즐 down" 쪽 상태로 끝나는 op인가 (MORE 목표 보정항).
    //   도색 move/arc  : true  - 노즐을 내린 채 끝나므로 중심이 a 앞
    //   오프셋 move(+a): true  - 곧 노즐을 내리려고 a 앞으로 나간 참이다
    //   오프셋 move(-a): false - 노즐을 올리고 물러났으므로 중심이 꼭짓점 위
    bool centerAheadByA = false;
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
                Pt tgt = {0, 0}, bool centerAheadByA = false) {
        OpMeta m;
        m.op = "move";
        m.isPath = isPath;
        m.headingDeg = m.exitHeadingDeg = headCcw;  // 직진은 방향이 안 바뀐다
        m.hasTarget = hasTgt;
        m.penTarget = tgt;
        m.centerAheadByA = centerAheadByA;
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
               bool hasTgt, Pt tgt, bool centerAheadByA) {
        OpMeta m;
        m.op = "arc";
        m.isPath = true;
        m.headingDeg = entryHeadCcw;
        m.exitHeadingDeg = exitHeadCcw;
        m.hasTarget = hasTgt;
        m.penTarget = tgt;
        m.centerAheadByA = centerAheadByA;
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
    // 노즐을 올릴 때 뒤로 물러날 방향 = 직전 도색 op의 **차체** 종료 방위.
    // 로봇은 회전하지 않았으므로 여전히 그쪽을 바라보고 있다. 호에서는 접선이
    // 아니라 접선 + φ다 - 접선으로 후진하면 중심이 꼭짓점에 안 돌아온다.
    double penBodyHeading = kNoHeading;
    // 지금 유지 중인 차체 위상. 도색을 끝낼 때 이만큼 되돌려 접선으로 세운다.
    double penPhase = 0.0;
    // 직전 도색 op이 끝나야 할 꼭짓점. closePaint의 후진 다리가 이 점을 목표로
    // 삼는다 (노즐 up 상태이므로 마커 중심이 그대로 꼭짓점 위여야 한다).
    Pt penEndTgt{0, 0};
    bool penEndHasTgt = false;

    // 도색 진입: (필요하면 위상만큼 틀고) a 전진 후 노즐 down.
    // 노즐을 올리는 op은 넣지 않는다 - penDown == false라 이미 올라가 있다.
    //
    // startTgt = 이 도색이 시작될 꼭짓점(= 펜이 내려앉아야 할 자리). 전진 다리에
    // 실어 보내면 그 다리가 끝난 boundary에서 MORE가 걸린다 - 목표는 "중심이
    // 꼭짓점보다 a 앞"(centerAheadByA=true)이라 곧 내려올 펜이 꼭짓점에 맞는다.
    // 🔴 이 보정은 노즐이 아직 올라가 있을 때 실행된다 (nozzle down은 다음 op).
    //   젖은 도료를 문지를 여지가 없다.
    auto openPaint = [&](const TravelGeom& g, bool hasStart, Pt startTgt) {
        if (g.phase != 0.0 && hasHeading(g.bodyEntry))
            seq.turnOp(g.phase, false, g.bodyEntry);  // 접선 -> 차체 방위
        seq.moveOp(+a, false, g.bodyEntry, hasStart, startTgt,
                   /*centerAheadByA=*/true);
        seq.nozzleOp(true);
        penDown = true;
        penPhase = g.phase;
    };
    // 도색 이탈: 노즐 up -> a 후진 -> 위상 원복. 원복까지 해야 뒤따르는 직선
    // op이 접선 기준으로 이어진다 (안 하면 이후 경로 전체가 φ만큼 기운다).
    //
    // 후진 다리도 직전 도색의 종료 꼭짓점을 목표로 물고 간다. 노즐을 이미 올린
    // 뒤라 목표는 꼭짓점 그대로다(centerAheadByA=false).
    // ⚠️ 경로 맨 끝의 closePaint는 뒤에 op이 없어 로봇이 READY 대신 PATH_DONE을
    //   보낸다 - 그 한 번만 보정이 걸리지 않는다. 도색은 이미 끝난 뒤라 무해하다.
    auto closePaint = [&]() {
        seq.nozzleOp(false);
        seq.moveOp(-a, false, penBodyHeading, penEndHasTgt, penEndTgt,
                   /*centerAheadByA=*/false);
        if (penPhase != 0.0 && hasHeading(penBodyHeading))
            seq.turnOp(-penPhase, false, normDeg(penBodyHeading - penPhase));
        penDown = false;
        penPhase = 0.0;
    };

    for (size_t i = 0; i < program.size(); ++i) {
        const json& q = program[i];
        std::string o = q.value("op", "");
        if (o == "NOZZLE") continue;  // 🔴 Qt NOZZLE은 전부 버린다 (§5.3)

        const bool isTravel = (o == "MOVE" || o == "ARC");
        const bool isPaint = isTravel && q.value("paint", false);
        const double head = q.value("heading_deg", kNoHeading);  // CCW 절대 방위
        const TravelGeom g = travelGeom(q, isPaint, a);

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

        // 출발 꼭짓점 = 이 op 자신의 v (validateProgram이 정수임을 보장한다).
        // 도색 진입 시 펜이 내려앉아야 할 자리라, 오프셋 전진 다리의 목표가 된다.
        Pt startTgt{0, 0};
        bool hasStartTgt = false;
        const int v0 = q.value("v", -1);
        if (v0 >= 0 && v0 < (int)points.size()) {
            startTgt = points[(size_t)v0];
            hasStartTgt = true;
        }

        // 🔴 위상이 바뀌는 것도 "도색 상태가 바뀌는 것"과 똑같이 취급한다.
        //   펜을 내린 채로 제자리 회전하면 펜이 반지름 a의 호를 그어 버린다 -
        //   직선에서 호로 넘어갈 때 φ만큼 트는 것도 예외가 아니다. 그래서
        //   접합부에서는 노즐을 반드시 한 번 올렸다 내린다.
        if (isPaint) {
            if (!penDown) {
                openPaint(g, hasStartTgt, startTgt);
            } else if (g.phase != penPhase) {
                closePaint();
                openPaint(g, hasStartTgt, startTgt);
            }
        } else if (penDown) {  // 도색 이탈 (turn / 비도색 이동 직전)
            closePaint();
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
            // 정상 흐름에서는 validateProgram이 이미 걸렀다. 여기서 한 번 더
            // 보는 것은 sqrt 안이 음수가 되어 NaN이 로봇까지 흘러가는 경로를
            // 원천 차단하기 위한 것 (§4.4 마이그레이션 체크리스트 #3).
            // 반지름 보정 자체는 travelGeom이 이미 했다 (g.rRobot).
            if (isPaint && rPaint < params().min_paint_radius_m) {
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
            // 🔴 arcOp에 넘기는 진입/출구 방위는 접선이 아니라 **차체 방위**다
            //   (도색 호면 접선 + φ). ALIGN 목표와 MORE 투영축이 전부 이 값을
            //   쓰는데, 로봇이 실제로 바라보는 방향은 차체 방위이기 때문이다.
            seq.arcOp(g.rRobot, angMag, dir, rPaint, g.bodyEntry, g.bodyExit,
                      hasTgt, tgt, isPaint);
        }
        // 다음 closePaint가 쓸 "직전 도색의 끝" 상태를 갱신한다. 방위는 후진
        // 방향, 꼭짓점은 후진 다리의 MORE 목표가 된다.
        if (isPaint) {
            if (hasHeading(g.bodyExit)) penBodyHeading = g.bodyExit;
            penEndTgt = tgt;
            penEndHasTgt = hasTgt;
        }
    }

    if (penDown) closePaint();  // 경로 끝 - 노즐 up + 중심 원복 + 위상 원복
    return seq.take();
}

// 도색을 시작할 때 로봇이 서 있어야 할 방위 = 첫 주행 op의 **진입 접선**.
// 접근 경로의 마지막 회전이 이 방향으로 세워두고, 도색 경로의 맨 앞 오프셋
// 보정(필요하면 turn(φ) 포함)이 거기서부터 이어받는다.
//
// heading_deg가 ARC에서도 진입 접선이므로(travelGeom 위 주석) 첫 주행 op의 값을
// 그대로 쓰면 된다. 위상 φ는 여기서 더하지 않는다 - 그건 도색 경로 맨 앞의
// turn(φ)가 담당한다. 접근 단계는 접선까지만 세워두고 넘긴다.
inline double firstEntryHeading(const json& program) {
    for (const auto& q : program) {
        const double h = q.value("heading_deg", kNoHeading);
        if (hasHeading(h)) return h;
    }
    return kNoHeading;
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
                   /*centerAheadByA=*/false);
        th = desired;
    }
    if (hasHeading(firstHeadingDeg)) {
        double turn = normDeg(firstHeadingDeg - th);
        if (std::fabs(turn) > P.min_turn_deg)
            seq.turnOp(turn, true, round1(firstHeadingDeg));
    }
    return seq.take();
}

// ===== 로봇 오도메트리 주행 캘리 경로 (2026-08-12) =====
// 규격: docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md §3.
//
// 닫힌 직사각형을 4변으로, 각 변을 반으로 쪼개 11개 op으로 돈다:
//   MOVE(m/2) MOVE(m/2) TURN(±90) MOVE(n/2) MOVE(n/2) TURN(±90)
//   MOVE(m/2) MOVE(m/2) TURN(±90) MOVE(n/2) MOVE(n/2)
// ccw=true(start_corner=="bottom_left")면 TURN(+90), false(top_left)면 TURN(-90).
//
// 🔴 전부 hasTarget=false, headingDeg=kNoHeading, centerAheadByA=false로
//   나간다 - MORE/ALIGN/DRIFT가 op 자체에서부터 차단된다 (isPath=false라
//   needsAlign의 role 검사, needsMore의 hasTarget 검사, DRIFT의 isPath 검사에
//   전부 걸린다). phase=="calib" 가드(onCalibReady)는 이중 방어다.
// 노즐 op은 넣지 않는다 - 도색하지 않는다.
inline PlannedPath buildCalibRectOps(double mM, double nM, bool ccw) {
    OpSeq seq;
    const double turnDeg = ccw ? 90.0 : -90.0;
    for (int leg = 0; leg < 4; ++leg) {
        const double half = (leg % 2 == 0 ? mM : nM) / 2.0;
        seq.moveOp(half, false, kNoHeading);
        seq.moveOp(half, false, kNoHeading);
        if (leg < 3) seq.turnOp(turnDeg, false, kNoHeading);
    }
    return seq.take();
}

// READY(op_index) -> point_index (0..7, 8은 PATH_DONE에서 별도 처리).
// -1 = 직전 op이 TURN이라 같은 물리 위치 (캡처 스킵, 즉시 GO).
// buildCalibRectOps()의 고정 11-op 구조(op 2/5/8이 TURN)에서 나온 표라
// m/n 값과 무관하다. docs/ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md §4.
inline int odoReadyToPoint(int k) {
    static constexpr int kTable[11] = {0, 1, 2, -1, 3, 4, -1, 5, 6, -1, 7};
    return (k >= 0 && k < 11) ? kTable[k] : -2;  // -2 = 범위 밖 (호출측 방어 처리용)
}

// point_index(0~8) -> "그 지점에서 로봇이 걷고 있(었)던 변" 번호(0~3).
// sendCalibCapture()의 마커 오프셋 보정에서 헤딩을 정하는 데 쓴다.
inline int odoLegOfPoint(int pointIdx) {
    static constexpr int kLeg[9] = {0, 0, 0, 1, 1, 2, 2, 3, 3};
    return kLeg[pointIdx >= 0 && pointIdx <= 8 ? pointIdx : 0];
}

// point_index(0~8)의 물리좌표(mm) - 마커 중심 기준, 오프셋 보정 포함.
// wire 스펙 §4 좌표표를 y0/ySign 하나의 공식으로 통일한다:
//   bottom_left(ccw=true) : y0=0,   ySign=+1  (0,0)에서 시작해 위로 올라간다
//   top_left   (ccw=false): y0=n,   ySign=-1  (0,n)에서 시작해 아래로 내려간다
// markerOffsetM이 0이 아니면 그 지점의 헤딩 방향으로 오프셋만큼 이동시킨다
// (§3-0 - 마커 중심 ≠ 회전 중심인 경우의 보정. 기본 0이면 좌표가 그대로 나간다).
inline std::array<double, 2> odoPointWorldMm(int pointIdx, double mMm, double nMm,
                                             bool ccw, double markerOffsetM = 0.0) {
    const double y0 = ccw ? 0.0 : nMm;
    const double ySign = ccw ? 1.0 : -1.0;
    double x = 0.0, y = y0;
    switch (pointIdx) {
        case 0: x = 0.0;       y = y0;                     break;
        case 1: x = mMm / 2.0; y = y0;                     break;
        case 2: x = mMm;       y = y0;                     break;
        case 3: x = mMm;       y = y0 + ySign * nMm / 2.0;  break;
        case 4: x = mMm;       y = y0 + ySign * nMm;        break;
        case 5: x = mMm / 2.0; y = y0 + ySign * nMm;        break;
        case 6: x = 0.0;       y = y0 + ySign * nMm;        break;
        case 7: x = 0.0;       y = y0 + ySign * nMm / 2.0;  break;
        default: x = 0.0;      y = y0;                      break;  // 8 = 복귀(진단용)
    }
    const double offsetMm = markerOffsetM * 1000.0;
    if (offsetMm != 0.0) {
        const int leg = odoLegOfPoint(pointIdx);
        const double headingDeg = normDeg(ccw ? 90.0 * leg : -90.0 * leg);
        const double rad = headingDeg * M_PI / 180.0;
        x += offsetMm * std::cos(rad);
        y += offsetMm * std::sin(rad);
    }
    return {x, y};
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

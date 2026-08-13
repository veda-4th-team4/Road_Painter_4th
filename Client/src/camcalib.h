#ifndef CAMCALIB_H
#define CAMCALIB_H

// 카메라 내부 파라미터(K)와 렌즈 왜곡 계수만 담는 값 객체 + 점 단위 변환.
//
// ⚠️ 이 파일은 **점 하나를 옮기는 계산만** 한다. 프레임(cv::Mat / QImage)은 절대
//    건드리지 않는다. RTSP 원본을 그대로 두고 좌표만 보정하는 것이 CCTV 와 합의한
//    규약이다 (양쪽이 각자 한 번씩만 보정 → 이중 적용이 생기지 않는다).
//
// 왜 필요한가: 카메라가 광각이라 바닥의 직선이 화면에서 활처럼 휜다. 호모그래피는
// 직선을 직선으로만 보내는 변환이라 이 곡률을 **원리적으로** 표현할 수 없다.
// 현장 실측(마커 16개): 왜곡 보정 없이 16~17mm, 보정하면 2.0mm. 8배 차이다.
//
// ⚠️ H 는 **왜곡 보정된 픽셀**을 받는다 (cv::undistortPoints 에 P=K 를 준 것과 같은 공간).
//    원본 픽셀을 그대로 넣으면 형상 오차가 4.3mm → 52.4mm 로 12배 뛴다. 실측값이다.
//    CCTV 도 `cv::undistortPoints(in, mapped, K, D, cv::noArray(), K)` 로 같은 공간을 쓴다.

#include <QJsonObject>
#include <QJsonArray>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QHash>
#include <cmath>

namespace camcalib {

// 3x3 호모그래피가 실제로 쓸 수 있는 값인가.
// 저장·적용 **전에** 통과해야 하는 최소 조건이다. 요청하지 않은/늦게 온 번들도
// 같은 검사를 받아야 화면 좌표계가 조용히 갈아엎히지 않는다.
//   · 3행 3열이어야 하고
//   · 모든 원소가 숫자이며 유한해야 하고
//   · 행렬식이 0 이 아니어야 한다 (역변환 불가 = 좌표를 만들 수 없다)
inline bool hasUsable3x3(const QJsonArray &h)
{
    if (h.size() != 3) return false;
    double m[3][3];
    for (int r = 0; r < 3; ++r) {
        if (!h.at(r).isArray()) return false;
        const QJsonArray row = h.at(r).toArray();
        if (row.size() != 3) return false;
        for (int c = 0; c < 3; ++c) {
            const QJsonValue cell = row.at(c);
            if (!cell.isDouble()) return false;
            m[r][c] = cell.toDouble();
            if (!std::isfinite(m[r][c])) return false;
        }
    }
    const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                     - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                     + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    return std::isfinite(det) && std::abs(det) > 1e-12;
}

// ── 캘리브레이션 번들 수용 규칙 (순수 함수 — 단위 테스트 대상) ────────────────
// 서버가 주는 채널 번들은 저장 **전에** 여기를 통과해야 한다.
//   · H / H_floor 가 있으면 반드시 쓸 수 있는 3x3 이어야 한다.
//   · H 가 전혀 없는 수동 코너 입력 번들(corners)은 기존 경로 그대로 허용한다.
inline QJsonArray floorHOf(const QJsonObject &raw)
{
    QJsonObject obj = raw;
    if (obj.contains(QStringLiteral("calib")) && obj.value(QStringLiteral("calib")).isObject())
        obj = obj.value(QStringLiteral("calib")).toObject();
    const QJsonArray h = obj.value(QStringLiteral("H")).toArray();
    return h.isEmpty() ? obj.value(QStringLiteral("H_floor")).toArray() : h;
}

inline bool declaresH(const QJsonObject &raw)
{
    QJsonObject obj = raw;
    if (obj.contains(QStringLiteral("calib")) && obj.value(QStringLiteral("calib")).isObject())
        obj = obj.value(QStringLiteral("calib")).toObject();
    return obj.contains(QStringLiteral("H")) || obj.contains(QStringLiteral("H_floor"));
}

inline bool hasCorners(const QJsonObject &raw)
{
    QJsonObject obj = raw;
    if (obj.contains(QStringLiteral("calib")) && obj.value(QStringLiteral("calib")).isObject())
        obj = obj.value(QStringLiteral("calib")).toObject();
    if (obj.value(QStringLiteral("corners")).toArray().size() >= 4) return true;
    for (const char *key : { "c0", "c1", "c2", "c3" }) {
        const QJsonValue value = obj.value(QLatin1String(key));
        if (!value.isObject() || value.toObject().isEmpty()) return false;
    }
    return true;
}

inline bool calibIsUsable(const QJsonObject &raw)
{
    if (raw.isEmpty()) return false;
    if (declaresH(raw)) return hasUsable3x3(floorHOf(raw));
    return hasCorners(raw);            // 수동 코너 전용 번들 (legacy·정상 경로)
}

// CH1~CH4 맵에서 쓸 수 있는 항목만 남긴다. rejected 에는 버린 키를 담는다.
inline QJsonObject filterUsableCalibMap(const QJsonObject &calibs, QStringList *rejected = nullptr)
{
    QJsonObject out;
    for (auto it = calibs.constBegin(); it != calibs.constEnd(); ++it) {
        const QJsonObject one = it.value().toObject();
        if (calibIsUsable(one)) out.insert(it.key(), it.value());
        else if (rejected) rejected->append(it.key());
    }
    return out;
}

// ── H_MATRIX 응답 처리 규칙 (순수 함수 — 단위 테스트 대상) ──────────────────
//   Apply     : 지금 기다리던 결과 → 저장 + 화면 적용
//   StoreOnly : 관리자/CCTV가 시작한 채널 확정 비동기 결과 또는 legacy 푸시
//   Drop      : Qt가 발급한 request_id인데 현재 요청과 다른 늦은 결과
enum class ReplyUse { Apply, StoreOnly, Drop };

inline bool isQtOwnedRequestId(const QString &requestId)
{
    return requestId.startsWith(QLatin1String("qt-"));
}

// 채널이 없을 때는 현재 하나뿐인 pending 요청과 request_id가 모순되지 않을 때만
// 그 pending 채널로 귀속한다. 0은 채널을 안전하게 확정할 수 없다는 뜻이다.
inline int resolveHomographyChannel(int ch, const QString &requestId,
                                    bool pending, int pendingCh,
                                    const QString &pendingRequestId,
                                    int channelCount = 4)
{
    if (ch >= 1 && ch <= channelCount) return ch;
    if (ch != 0) return 0;
    if (!pending || pendingCh < 1 || pendingCh > channelCount) return 0;
    if (!requestId.isEmpty() && requestId != pendingRequestId) return 0;
    return pendingCh;
}

// ── 오도메트리 주행 캘리브레이션 규칙 (순수 함수 — 단위 테스트 대상) ──────────
// 사각형 각 변의 절반이 서버 min_move_m(1cm) 이상이어야 하므로 변은 2cm 이상.
// 상한은 서버가 1000cm(10m)로 못 박았다 — 그보다 큰 값은 서버가 invalid_param
// 으로 거절하므로 여기서 먼저 막는다. 양끝(2cm·1000cm)은 **포함**이다.
// NaN/무한/빈 입력이 그대로 JSON 에 실리면 서버가 invalid_param 으로 거절하거나
// 로봇이 엉뚱한 거리를 주행한다 — 전송 전에 여기서 막는다.
inline constexpr double kOdoSizeMinCm = 2.0;
inline constexpr double kOdoSizeMaxCm = 1000.0;

inline bool odoSizeValidCm(double cm)
{
    return std::isfinite(cm) && cm >= kOdoSizeMinCm && cm <= kOdoSizeMaxCm;
}

// 🔴 화면에 보이는 글자 그대로를 검사한다. 예전에는 QML validator 가 범위를 벗어난
//    입력의 editingFinished 를 막아서, 화면에는 1200 이 떠 있는데 시작 버튼은 직전
//    유효값(예: 90)을 보냈다 — 조작자가 본 거리와 로봇이 도는 거리가 달라진다.
//    빈 칸·문자·꼬리 문자("90cm")·NaN/inf 는 전부 실패다. 앞뒤 공백만 허용한다.
inline bool parseOdoSizeCm(const QString &text, double *out = nullptr)
{
    const QString s = text.trimmed();
    if (s.isEmpty()) return false;
    bool ok = false;
    const double v = s.toDouble(&ok);      // 부분 파싱을 하지 않는다 (꼬리 문자 = 실패)
    if (!ok || !odoSizeValidCm(v)) return false;
    if (out) *out = v;
    return true;
}

// 두 입력의 검증 결과를 한 문장으로. Backend 통지와 단위 테스트가 같은 문구를 쓴다.
inline QString odoSizeRangeText()
{
    return QStringLiteral("가로·세로 값을 확인하세요 (각 %1cm 이상 %2cm 이하의 숫자).")
        .arg(kOdoSizeMinCm, 0, 'g', 3).arg(kOdoSizeMaxCm, 0, 'g', 4);
}

// UI 의 좌/우회전 → 서버 필드값. 🔴 이름을 바꾸면 서버가 CALIB_START 원본을
// ROBOT/CCTV 로 그대로 중계하므로 양쪽이 못 읽는다 (요청서 §1).
inline QString odoStartCorner(bool ccw)
{
    return ccw ? QStringLiteral("bottom_left") : QStringLiteral("top_left");
}

// 캡처 지연 경고. point_index 는 0-based 이므로 **완료된 정지점 수 = index + 1**.
// 유효 대응점이 완료 정지점보다 2 이상 뒤처지면 카메라가 로봇을 놓치고 있다
// (6개 미만으로 끝나면 세션 실패). 값이 없으면(-1) 경고하지 않는다.
inline bool captureLagWarning(int pointIndex, int valid)
{
    if (pointIndex < 0 || valid < 0) return false;
    return (pointIndex + 1) - valid >= 2;
}

// 조작자가 중단을 요청한 세션의 결과는 **통째로 버린다.**
// 확인 대기 중이든(cancel pending) 확인을 못 받았든(unconfirmed) 동일하다 —
// 취소한 세션의 산출물을 채널 번들로 남기면 나중에 그 채널에 들어갈 때 조용히
// 적용되고, 조작자는 자기가 취소한 결과를 쓰고 있는 줄 모른다.
// 🔴 강등 대상은 **내 요청의 답(Apply)** 뿐이다. 관리자/CCTV 가 시작한 채널 확정
//    결과(StoreOnly)는 내 취소와 무관하므로 라우팅을 바꾸지 않는다.
inline ReplyUse afterCancelRequest(ReplyUse use, bool cancelRequested)
{
    if (!cancelRequested) return use;
    return (use == ReplyUse::Apply) ? ReplyUse::Drop : use;
}

// 외부(서버/CCTV/관리자 창) 결과를 **지금 보고 있는 화면**에 즉시 반영해야 하는가.
// CH1~CH4 대칭 규칙: 지금 작업 화면의 채널과 같을 때만 TopView 를 갈아끼우고,
// 다른 채널이면 화면을 건드리지 않고 채널 맵에만 저장한다.
inline bool appliesToCurrentView(int ch, int workingCh)
{
    return ch >= 1 && ch == workingCh;
}

// CALIB_FAIL 사유별 조작자 문구 (요청서 §2-3). 모르는 reason 은 서버 msg 를
// 그대로 띄운다 — 카메라 쪽 사유가 계속 늘어난다.
inline QString homographyFailText(const QString &reason, const QString &message,
                                    const QString &owner)
{
    if (reason == QLatin1String("busy")) {
        if (owner == QLatin1String("ADMIN"))
            return QStringLiteral("관리자 창에서 캘리브레이션이 진행 중입니다.");
        if (owner == QLatin1String("QT"))
            return QStringLiteral("이미 캘리브레이션이 진행 중입니다.");
        return message.isEmpty() ? QStringLiteral("다른 작업이 진행 중이라 시작할 수 없습니다.")
                                 : message;
    }
    static const QHash<QString, QString> kText = {
        { QStringLiteral("invalid_param"),
          QStringLiteral("가로·세로 값을 확인하세요 (각 2cm 이상 1000cm 이하).") },
        { QStringLiteral("too_few_points"),
          QStringLiteral("카메라가 로봇을 충분히 인식하지 못했습니다. 조명과 사각형 크기를 "
                         "확인하세요.") },
        { QStringLiteral("fit_failed"),
          QStringLiteral("호모그래피 계산에 실패했습니다. 다시 시도하세요.") },
        { QStringLiteral("no_intrinsics"),
          QStringLiteral("카메라 내부 보정(체커보드)을 먼저 완료하세요.") },
        { QStringLiteral("capture_timeout"),
          QStringLiteral("카메라 응답이 없어 중단했습니다.") },
        { QStringLiteral("preempted"),
          QStringLiteral("관리자가 캘리브레이션을 중단했습니다.") },
        { QStringLiteral("not_owner"),
          QStringLiteral("이 캘리브레이션 세션의 소유자가 아닙니다.") },
        { QStringLiteral("invalid_channel"),
          QStringLiteral("서버가 채널을 인식하지 못했습니다. 채널을 다시 선택하세요.") },
        { QStringLiteral("robot_offline"), QStringLiteral("로봇이 연결되어 있지 않습니다.") },
        { QStringLiteral("cctv_offline"), QStringLiteral("카메라가 연결되어 있지 않습니다.") },
    };
    const auto it = kText.constFind(reason);
    if (it != kText.constEnd()) return it.value();
    return message.isEmpty()
               ? QStringLiteral("호모그래피 계산에 실패했습니다 (%1).").arg(reason)
               : message;     // 모르는 사유는 서버 문구를 그대로
}

inline ReplyUse classifyHomographyReply(int ch, const QString &requestId,
                                        bool pending, int pendingCh,
                                        const QString &pendingRequestId)
{
    if (!requestId.isEmpty()) {
        if (pending && ch == pendingCh && requestId == pendingRequestId)
            return ReplyUse::Apply;
        if (isQtOwnedRequestId(requestId)) return ReplyUse::Drop;
        return ReplyUse::StoreOnly;
    }
    // legacy: 구형 서버는 request_id 를 돌려주지 않는다. 같은 채널을 기다리는
    // 중이면 그 결과로 인정하고, 아니면 저장만 한다. 어느 쪽이든 행렬 검증은 별도로 한다.
    if (pending && ch == pendingCh) return ReplyUse::Apply;
    return ReplyUse::StoreOnly;
}

struct Model {
    bool   valid = false;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    // OpenCV 표준 plumb-bob 순서: [k1, k2, p1, p2, k3]
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;
    int    imgW = 0, imgH = 0;
    QString coordMode;          // "undistort" 만 허용한다 (아래 parse 참고)
    QString calibId;

    bool hasDistortion() const {
        return k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0;
    }

    // ⚠️ 여기 있던 undistort(원본 픽셀 → 보정 픽셀, 고정점 반복 20회)는 지웠다.
    //    호출부가 0 이다. TopView 배경 보정은 OpenCV 의 remap 이 하고, 좌표 변환은
    //    아래 distort 로 **반대 방향**만 쓴다:
    //        TopView px → (m_tvHinv) → 보정 픽셀 → (distort) → 원본 RTSP px
    //    보정 픽셀에서 출발하므로 역변환이 필요한 자리가 없다.

    // 왜곡 보정 픽셀 → 원본(왜곡된) 픽셀. 정방향이라 반복 없이 한 번에 끝난다.
    // 월드 좌표를 원본 영상 위에 그릴 때 쓴다.
    QPointF distort(const QPointF &ud) const {
        if (!valid || !hasDistortion()) return ud;
        const double x  = (ud.x() - cx) / fx;
        const double y  = (ud.y() - cy) / fy;
        const double r2 = x * x + y * y;
        const double rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const double xd = x * rad + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double yd = y * rad + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        return QPointF(fx * xd + cx, fy * yd + cy);
    }
};

// 3x3 배열을 [[a,b,c],[d,e,f],[g,h,i]] 또는 [a,b,c,d,e,f,g,h,i] 양쪽 형태로 읽는다.
inline bool readMat3(const QJsonValue &v, double out[9])
{
    const QJsonArray a = v.toArray();
    if (a.size() == 9) {
        for (int i = 0; i < 9; ++i) out[i] = a.at(i).toDouble();
        return true;
    }
    if (a.size() == 3) {
        for (int r = 0; r < 3; ++r) {
            const QJsonArray row = a.at(r).toArray();
            if (row.size() != 3) return false;
            for (int c = 0; c < 3; ++c) out[r * 3 + c] = row.at(c).toDouble();
        }
        return true;
    }
    return false;
}

// 캘리브레이션 번들에서 K/dist 를 읽는다.
//   - K   : 3x3 (중첩·평탄 배열 모두 허용) 또는 {"fx","fy","cx","cy"} 객체
//   - 왜곡: "D" 또는 "dist" 배열 (4개 또는 5개)
//   - "image_size" / "imageSize" : [w, h]
//   - "coord_mode" : "undistort" 가 아니면 **거부**한다
//
// 실패해도 예외를 던지지 않는다. valid=false 로 두고 err 에 이유를 채운다 —
// 캘리브레이션이 없어도 작도는 되어야 하기 때문이다.
inline Model parse(const QJsonObject &objIn, QString *err = nullptr)
{
    Model m;
    auto fail = [&](const QString &why) { if (err) *err = why; return m; };

    // {"calib": {...}} 로 감싸 오는 형태도 그대로 받는다.
    QJsonObject o = objIn;
    if (o.contains(QStringLiteral("calib")) && o.value(QStringLiteral("calib")).isObject())
        o = o.value(QStringLiteral("calib")).toObject();

    if (!o.contains(QStringLiteral("K")))
        return fail(QStringLiteral("K 없음"));

    const QJsonValue kv = o.value(QStringLiteral("K"));
    if (kv.isObject()) {
        const QJsonObject ko = kv.toObject();
        m.fx = ko.value(QStringLiteral("fx")).toDouble();
        m.fy = ko.value(QStringLiteral("fy")).toDouble();
        m.cx = ko.value(QStringLiteral("cx")).toDouble();
        m.cy = ko.value(QStringLiteral("cy")).toDouble();
    } else {
        double K[9] = {0};
        if (!readMat3(kv, K)) return fail(QStringLiteral("K 형식이 3x3 이 아님"));
        m.fx = K[0]; m.cx = K[2];
        m.fy = K[4]; m.cy = K[5];
    }
    if (!(m.fx > 1.0) || !(m.fy > 1.0))
        return fail(QStringLiteral("K 의 fx/fy 가 비정상 (%1, %2)").arg(m.fx).arg(m.fy));

    QJsonArray d = o.value(QStringLiteral("D")).toArray();
    if (d.isEmpty()) d = o.value(QStringLiteral("dist")).toArray();
    if (d.size() >= 4) {
        m.k1 = d.at(0).toDouble(); m.k2 = d.at(1).toDouble();
        m.p1 = d.at(2).toDouble(); m.p2 = d.at(3).toDouble();
        m.k3 = (d.size() >= 5) ? d.at(4).toDouble() : 0.0;
    } else if (!d.isEmpty()) {
        return fail(QStringLiteral("왜곡 계수가 %1개 (4 또는 5개여야 함)").arg(d.size()));
    }

    QJsonArray sz = o.value(QStringLiteral("image_size")).toArray();
    if (sz.isEmpty()) sz = o.value(QStringLiteral("imageSize")).toArray();
    if (sz.size() >= 2) {
        m.imgW = sz.at(0).toInt();
        m.imgH = sz.at(1).toInt();
    }

    // ⚠️ 카메라에는 HG_COORD_MODE raw|undistort 스위치가 있고, H 가 어느 픽셀 공간에서
    //    피팅됐는지가 같이 저장된다. raw 로 맞춘 H 를 이 파이프라인에 넣으면 조용히
    //    52mm 틀린다 — 그래서 모르면 통과시키되, raw 라고 **명시**되면 거부한다.
    m.coordMode = o.value(QStringLiteral("coord_mode")).toString();
    if (!m.coordMode.isEmpty() && m.coordMode.compare(QStringLiteral("undistort"),
                                                      Qt::CaseInsensitive) != 0)
        return fail(QStringLiteral("coord_mode 가 '%1' 입니다 — undistort 만 지원합니다")
                        .arg(m.coordMode));

    m.calibId = o.value(QStringLiteral("calib_id")).toString();
    m.valid = true;
    if (err) err->clear();
    return m;
}

// ── 왜곡 보정 데이터 누락 판정 (순수 함수 — 단위 테스트 대상) ────────────────
// 번들이 coord_mode="undistort" 라고 **선언**했다면 H 는 왜곡 보정된 픽셀을 받는다.
// 그런데 그 보정을 실제로 수행할 K 또는 D(dist) 가 번들에 없으면, 화면은 원본
// 픽셀을 그대로 쓰면서 좌표만 보정된 척하게 된다 (실측 4.3mm → 52.4mm).
// 번들 자체는 기존과 똑같이 수용하되(H 만 유효하면 쓸 수 있다), 조작자에게는
// "이 채널은 왜곡 보정 데이터가 없다"고 반드시 알려야 한다.
inline QString coordModeOf(const QJsonObject &raw)
{
    QJsonObject o = raw;
    if (o.contains(QStringLiteral("calib")) && o.value(QStringLiteral("calib")).isObject())
        o = o.value(QStringLiteral("calib")).toObject();
    QString mode = o.value(QStringLiteral("coord_mode")).toString();
    if (mode.isEmpty()) mode = o.value(QStringLiteral("coordMode")).toString();
    return mode;
}

inline bool declaresUndistort(const QJsonObject &raw)
{
    return coordModeOf(raw).compare(QStringLiteral("undistort"), Qt::CaseInsensitive) == 0;
}

// K 를 실제로 읽을 수 있는가 (parse 와 같은 기준 — fx/fy 가 정상이어야 한다).
inline bool hasUsableK(const QJsonObject &raw)
{
    QJsonObject o = raw;
    if (o.contains(QStringLiteral("calib")) && o.value(QStringLiteral("calib")).isObject())
        o = o.value(QStringLiteral("calib")).toObject();
    if (!o.contains(QStringLiteral("K"))) return false;
    QJsonObject probe;
    probe.insert(QStringLiteral("K"), o.value(QStringLiteral("K")));
    const Model m = parse(probe);          // coord_mode 없이 K 만 본다
    return m.valid;
}

// D(또는 dist) 가 plumb-bob 4/5 계수로 들어 있는가.
inline bool hasUsableD(const QJsonObject &raw)
{
    QJsonObject o = raw;
    if (o.contains(QStringLiteral("calib")) && o.value(QStringLiteral("calib")).isObject())
        o = o.value(QStringLiteral("calib")).toObject();
    QJsonArray d = o.value(QStringLiteral("D")).toArray();
    if (d.isEmpty()) d = o.value(QStringLiteral("dist")).toArray();
    if (d.size() != 4 && d.size() != 5) return false;
    for (const QJsonValue &v : d)
        if (!v.isDouble() || !std::isfinite(v.toDouble())) return false;
    return true;
}

// 채널별 통지 키. 채널마다 달라야 한 채널의 경고가 다른 채널 경고를 지우지 않는다.
// (ch < 1 = 채널을 알 수 없는 legacy 단일 번들)
inline QString lensDataNoticeKey(int ch)
{
    return ch >= 1 ? QStringLiteral("calib-lensdata-CH%1").arg(ch)
                   : QStringLiteral("calib-lensdata");
}

// undistort 를 선언했는데 K 또는 D 가 없거나 못 읽는 번들인가.
// raw/빈 coord_mode 는 이 경고의 대상이 아니다 (그쪽은 보정을 쓰지 않는다).
inline bool lensDataMissingForUndistort(const QJsonObject &raw)
{
    if (raw.isEmpty()) return false;
    if (!declaresUndistort(raw)) return false;
    return !hasUsableK(raw) || !hasUsableD(raw);
}

// 사람이 읽는 한 줄 요약. 설정 화면과 로그에 같은 문자열을 쓴다.
inline QString describe(const Model &m)
{
    if (!m.valid) return QStringLiteral("렌즈 보정 없음");
    QStringList parts;
    parts << QStringLiteral("fx %1 · cx %2").arg(m.fx, 0, 'f', 1).arg(m.cx, 0, 'f', 1);
    parts << (m.hasDistortion() ? QStringLiteral("k1 %1").arg(m.k1, 0, 'f', 4)
                                : QStringLiteral("왜곡계수 0"));
    if (m.imgW > 0) parts << QStringLiteral("%1×%2").arg(m.imgW).arg(m.imgH);
    if (!m.calibId.isEmpty()) parts << m.calibId;
    return parts.join(QStringLiteral(" · "));
}

} // namespace camcalib

#endif // CAMCALIB_H

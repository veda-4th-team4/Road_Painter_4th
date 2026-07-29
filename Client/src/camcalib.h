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
#include <cmath>

namespace camcalib {

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

    // 원본(왜곡된) 픽셀 → 왜곡 보정 픽셀. cv::undistortPoints(..., P=K) 와 같은 결과.
    // 왜곡 식은 정방향(보정→왜곡)만 닫힌 형태라, 역방향은 고정점 반복으로 푼다.
    // 20회면 이 렌즈 계수 범위에서 픽셀 이하로 수렴한다.
    QPointF undistort(const QPointF &raw) const {
        if (!valid || !hasDistortion()) return raw;
        const double x0 = (raw.x() - cx) / fx;
        const double y0 = (raw.y() - cy) / fy;
        double x = x0, y = y0;
        for (int i = 0; i < 20; ++i) {
            const double r2  = x * x + y * y;
            const double rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            if (std::abs(rad) < 1e-12) break;
            const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (x0 - dx) / rad;
            y = (y0 - dy) / rad;
        }
        return QPointF(fx * x + cx, fy * y + cy);
    }

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

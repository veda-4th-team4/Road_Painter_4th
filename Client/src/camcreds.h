#ifndef CAMCREDS_H
#define CAMCREDS_H
// ── 카메라 자격증명 — **소스에 두지 않는다** ─────────────────────────────────
//
// 이 저장소는 공개(public)다. 예전에는 RTSP URL 에 계정과 비밀번호가 그대로
// 박혀 있어서 누구나 읽을 수 있었다. 이제 URL 템플릿은 `{user}` / `{pass}` 자리
// 표시자만 갖고, 실제 값은 실행 시점에 여기서 채운다.
//
// 읽는 순서:
//   ① 환경변수 RP_CAM_USER / RP_CAM_PASS
//   ② 실행 파일 옆의 `camera.env`  (KEY=VALUE 한 줄씩, .gitignore 대상)
//   ③ 없으면 빈 값 → 호출부가 접속을 시도하지 않는다
//
// 🔴 값이 없을 때 빈 문자열로 접속하면 안 된다. 한화 카메라는 틀린 자격증명으로
//    반복 접속하면 계정을 잠근다(RTSP/1.0 490 Account Blocked). 그래서 호출부는
//    available() 이 false 면 URL 자체를 만들지 않는다.
//
// ⚠️ 과거 커밋에는 옛 비밀번호가 그대로 남아 있다. 이 파일을 넣는다고 그게
//    사라지지 않는다 — **카메라 비밀번호를 반드시 교체해야** 의미가 있다.
#include <QByteArray>
#include <QCoreApplication>
#include <QFile>
#include <QHash>
#include <QString>
#include <QTextStream>

namespace camcreds {

// `camera.env` 를 한 번만 읽어 캐시한다. 파일이 없으면 빈 맵이다.
inline const QHash<QString, QString> &envFile()
{
    static const QHash<QString, QString> cached = [] {
        QHash<QString, QString> out;
        const QString path = QCoreApplication::applicationDirPath()
                           + QStringLiteral("/camera.env");
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return out;
        QTextStream in(&f);
        while (!in.atEnd()) {
            const QString line = in.readLine().trimmed();
            if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) continue;
            const int eq = line.indexOf(QLatin1Char('='));
            if (eq <= 0) continue;
            // 값의 앞뒤 따옴표는 벗긴다 — camera.env 를 편집기로 만들면 흔히 붙는다.
            QString v = line.mid(eq + 1).trimmed();
            if (v.size() >= 2
                && ((v.startsWith(QLatin1Char('"')) && v.endsWith(QLatin1Char('"')))
                 || (v.startsWith(QLatin1Char('\'')) && v.endsWith(QLatin1Char('\'')))))
                v = v.mid(1, v.size() - 2);
            out.insert(line.left(eq).trimmed(), v);
        }
        return out;
    }();
    return cached;
}

inline QString value(const QString &key)
{
    const QByteArray fromEnv = qgetenv(key.toUtf8().constData());
    if (!fromEnv.isEmpty()) return QString::fromUtf8(fromEnv);
    return envFile().value(key);
}

inline QString user() { return value(QStringLiteral("RP_CAM_USER")); }
inline QString pass() { return value(QStringLiteral("RP_CAM_PASS")); }

// 둘 다 있어야 접속을 시도한다 (위 계정 잠금 주석 참고).
inline bool available() { return !user().isEmpty() && !pass().isEmpty(); }

// URL 템플릿의 자리표시자를 실제 값으로 채운다.
inline QString apply(QString url)
{
    url.replace(QStringLiteral("{user}"), user());
    url.replace(QStringLiteral("{pass}"), pass());
    return url;
}

} // namespace camcreds

#endif // CAMCREDS_H

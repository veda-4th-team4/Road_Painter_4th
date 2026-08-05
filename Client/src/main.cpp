#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QFontDatabase>
#include <QFont>

#include "backend.h"
#include "video_worker.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // ⚠️ RTSP 캡처 옵션은 **여기서 딱 한 번** 세팅한다. qputenv 는 프로세스 전역이라
    //    워커(본 화면 1 + 미리보기 4)가 각자 부르면 서로 덮어쓴다. OpenCV 는 이 값을
    //    VideoCapture::open() 시점에 읽으므로, 워커가 만들어지기 전이면 된다.
    //    FFmpeg 버전별 타임아웃 옵션 이름 차이도 이 함수가 처리한다.
    video_worker::applyFfmpegCaptureOptions();
    app.setApplicationName("Road Painter");
    // QSettings 저장 위치를 고정한다 (펜 오프셋 등 기구 설정)
    app.setOrganizationName("RoadPainter");
    app.setOrganizationDomain("roadpainter.local");

    // Pretendard 폰트 등록 및 앱 전역 기본 폰트로 지정 (QML Text/Controls 모두 상속)
    const QStringList fontFiles = {
        ":/fonts/Pretendard-Regular.otf",
        ":/fonts/Pretendard-Medium.otf",
        ":/fonts/Pretendard-SemiBold.otf",
        ":/fonts/Pretendard-Bold.otf",
    };
    QString family;
    for (const QString &f : fontFiles) {
        int id = QFontDatabase::addApplicationFont(f);
        if (id != -1 && family.isEmpty()) {
            const QStringList fams = QFontDatabase::applicationFontFamilies(id);
            if (!fams.isEmpty()) family = fams.first();
        }
    }
    if (!family.isEmpty()) {
        QFont f(family);
        f.setStyleStrategy(QFont::PreferAntialias);
        app.setFont(f);
    }

    QQuickStyle::setStyle("Basic"); // 완전 커스텀 스타일링을 위해 Basic 사용

    QQmlApplicationEngine engine;

    Backend backend;
    engine.rootContext()->setContextProperty("Backend", &backend);

    engine.loadFromModule("RoadPainter", "Main");
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>

#include "filtersettings.h"
#include "pathmodel.h"
#include "presetlibrary.h"
#include "robotsimulator.h"

#include <QDir>
#include <QFile>
#include <QTimer>
#include <QUrl>
#include <cstdio>

// GUI 없이 백엔드 로직(경로 모델, JSON 내보내기/불러오기, 프리셋, 로봇 시뮬레이션)을
// 빠르게 검증하는 헤드리스 모드. `appRoadPainter --selftest` 로 실행한다.
static int runSelfTest()
{
    int failures = 0;
    const auto check = [&failures](bool ok, const char *name) {
        std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok)
            ++failures;
    };

    // 1. 경로 모델 기본 동작
    PathModel model;
    model.addPoint(400, 300);
    model.addPoint(800, 250);
    model.addPoint(1000, 500);
    check(model.count() == 3, "addPoint x3");
    check(qRound(model.totalLength()) == qRound(403.11 + 320.16), "totalLength");

    model.translateAll(50, -30);
    const QVariantList moved = model.points();
    check(qRound(moved[0].toMap()["x"].toDouble()) == 450
          && qRound(moved[0].toMap()["y"].toDouble()) == 270, "translateAll");
    model.translateAll(-50, 30);

    // 2. JSON 내보내기 (픽셀)
    const QUrl tmp = QUrl::fromLocalFile(QDir::temp().absoluteFilePath("roadpainter_selftest.json"));
    QVariantMap opts;
    opts["coordinateSpace"] = "pixel";
    opts["closed"] = true;
    check(model.exportToFile(tmp, opts), "exportToFile(pixel)");

    // 3. 다시 불러오기 (라운드트립)
    PathModel model2;
    check(model2.importFromFile(tmp), "importFromFile(BLUEPRINT)");
    check(model2.count() == 3, "roundtrip point count");
    check(model2.closed(), "roundtrip closed flag");
    check(qRound(model2.points()[2].toMap()["x"].toDouble()) == 1000, "roundtrip coords");

    // 4. 미터 좌표 내보내기 → 불러오기 시 픽셀 복원
    opts["coordinateSpace"] = "meter";
    check(model.exportToFile(tmp, opts), "exportToFile(meter)");
    PathModel model3;
    check(model3.importFromFile(tmp), "importFromFile(meter)");
    check(qAbs(model3.points()[1].toMap()["x"].toDouble() - 800.0) < 1.0, "meter->pixel restore");

    // 5. 레거시 스키마 불러오기
    const QString legacyPath = QDir::temp().absoluteFilePath("roadpainter_legacy.json");
    {
        QFile f(legacyPath);
        f.open(QIODevice::WriteOnly);
        f.write(R"({"path":[{"sequence":1,"x":350,"y":300},{"sequence":2,"x":900,"y":300}],"total_points":2})");
    }
    PathModel model4;
    check(model4.importFromFile(QUrl::fromLocalFile(legacyPath)), "importFromFile(legacy)");
    check(model4.count() == 2, "legacy point count");

    // 6. 프리셋 생성
    PresetLibrary presets;
    check(presets.generate("line", 640, 360, 150).size() == 2, "preset line");
    check(presets.generate("rectangle", 640, 360, 150).size() == 4, "preset rectangle");
    check(presets.generate("triangle", 640, 360, 150).size() == 3, "preset triangle");
    check(presets.generate("hexagon", 640, 360, 150).size() == 6, "preset hexagon");
    check(presets.generate("parking", 640, 360, 150).size() == 10, "preset parking");

    // 7. 로봇 시뮬레이션 (이벤트 루프에서 완주 확인)
    RobotSimulator robot;
    robot.setSpeed(500);
    QVariantList shortPath;
    shortPath.append(QVariantMap{ { "x", 100.0 }, { "y", 640.0 } });
    shortPath.append(QVariantMap{ { "x", 220.0 }, { "y", 640.0 } });
    check(robot.setPath(shortPath, false), "robot setPath");

    int trailCount = 0;
    bool robotFinished = false;
    QObject::connect(&robot, &RobotSimulator::trailSegment,
                     [&trailCount](qreal, qreal, qreal, qreal) { ++trailCount; });
    QObject::connect(&robot, &RobotSimulator::finished,
                     [&robotFinished]() { robotFinished = true; });
    check(robot.start(), "robot start");

    QTimer deadline;
    deadline.setSingleShot(true);
    QObject::connect(&deadline, &QTimer::timeout, QCoreApplication::instance(),
                     &QCoreApplication::quit);
    QObject::connect(&robot, &RobotSimulator::finished, QCoreApplication::instance(),
                     &QCoreApplication::quit);
    deadline.start(5000);
    QCoreApplication::exec();

    check(robotFinished, "robot finished");
    check(trailCount > 0, "robot trail segments emitted");
    check(qAbs(robot.x() - 220.0) < 0.5 && qAbs(robot.y() - 640.0) < 0.5, "robot final pose");
    check(robot.progress() >= 1.0, "robot progress 100%");

    std::printf("%s (%d failure(s))\n", failures == 0 ? "SELFTEST OK" : "SELFTEST FAILED", failures);
    return failures == 0 ? 0 : 1;
}

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QGuiApplication::setApplicationName(QStringLiteral("Road-Painter"));
    QGuiApplication::setOrganizationName(QStringLiteral("VEDA4th"));

    for (int i = 1; i < argc; ++i) {
        if (qstrcmp(argv[i], "--selftest") == 0)
            return runSelfTest();
    }

    // 커스텀 다크 테마를 직접 스타일링하므로 Basic 스타일을 기반으로 사용한다.
    QQuickStyle::setStyle(QStringLiteral("Basic"));

    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("RoadPainter", "Main");

    return QCoreApplication::exec();
}

#include "video_worker.h"
#include <QDebug>
#include <QRegularExpression>
#include <opencv2/objdetect/aruco_detector.hpp>

// RTSP 캡처 옵션을 프로세스 전역으로 한 번 세팅한다. main() 에서 **VideoCapture 를
// 만들기 전에** 부를 것 — OpenCV 는 이 환경변수를 open() 시점에 읽는다.
//
// ⚠️ qputenv 는 프로세스 전역이다. 워커가 여러 개(본 화면 1 + 미리보기 4)라 각
//    워커의 run() 에서 부르면 서로 덮어쓴다. 그래서 여기 한 곳으로 모았다.
//
// 담는 옵션 셋:
//
//  1) rtsp_transport;tcp
//     기본값(UDP)은 이 현장에서 30초쯤 타임아웃을 먹고서야 붙는다 — TCP 로는 4초면
//     첫 프레임이 나온다. 중계 서버(MediaMTX)도 rtspTransports:[tcp] 로 맞춰져 있다.
//
//  2) 소켓 읽기 타임아웃 5초
//     ⚠️ **옵션 이름이 FFmpeg 버전에 따라 다르다.** 이게 이 함수가 존재하는 이유다.
//        · FFmpeg 4.x (avformat 58): `stimeout` — 마이크로초 단위 소켓 I/O 타임아웃.
//          이 버전에서 `timeout` 은 뜻이 **전혀 다르다**. "들어오는 연결을 기다리는
//          시간"이라, 주는 순간 listen 플래그가 켜져 클라이언트가 아니라 **서버**로
//          동작하려 든다 → `Unable to open RTSP for listening` 으로 접속도 못 해본다.
//        · FFmpeg 5.0+ (avformat 59+): `stimeout` 이 `timeout` 으로 통합됐다.
//          옛 이름을 주면 **조용히 무시**되고 타임아웃이 기본값 30초로 돌아간다.
//          그러면 스트림이 죽었을 때 grab() 하나가 30초를 통째로 잡아먹고, 그동안
//          stop() 도 못 듣는다 → 주소를 바꾸면 옛 워커가 살아남아 디코더가 겹친다.
//     둘 다 넣으면 안 된다 — 하나는 무시되는 게 아니라 다른 뜻으로 먹힌다.
//     그래서 **실행 시점에 avformat 버전을 읽어** 맞는 이름 하나만 쓴다.
//     (OpenCV 를 4.12↔4.13 등으로 갈아끼우면 번들 FFmpeg 도 같이 바뀐다.)
//
//  3) flags;low_delay
//     디코더가 프레임을 재정렬하려고 붙잡아 두는 걸 막는다(H.264 프레임 스레딩은
//     출력을 N-1 장 늦춘다). 실측(2026-07-31)으로 쌓여 있던 깊이 최대 7프레임 —
//     30fps 면 약 230ms 가 통째로 화면 지연이었다. 첫 프레임이 조금 늦어지는 건
//     "첫 화면은 느려도 되고 그 뒤 실시간성이 중요"라는 결정에 맞는 교환이다.
//
//     ⚠️ fflags;nobuffer 는 **일부러 안 넣는다.** 사연이 있어서 근거를 남긴다.
//
//        · GOV 60(2초) 시절에는 이걸 넣으면 **프레임이 한 장도 안 나왔다**
//          (2026-08-04, 정순·역순 모두 0 fps · `cabac_init_idc overflow` →
//          `decode_slice_header error` → `no frame!`). nobuffer 가 디먹서의 선행
//          버퍼링을 없애서 SPS/PPS 를 못 물고 GOP 중간부터 디코딩을 시작한 탓이다.
//          파라미터셋을 놓치면 다음 IDR 까지 2초가 통째로 깨진다.
//        · **GOV 8 로 바꾼 뒤 재측정하니 더 이상 안 깨진다** — 15fps 기준 IDR 이
//          0.53초마다 오므로 놓쳐도 금방 복구된다 (정순 9.8fps / 역순 11.5fps).
//        · 그런데 **더 좋다는 증거가 없다.** 같은 측정에서 nobuffer 없는 쪽이
//          정순 8.5 / 역순 15.2 fps 로 회차마다 뒤집혔고(무선 구간 잡음),
//          nobuffer 쪽만 두 번 다 디코딩 실패 3건이 붙었다.
//        결론: 이득이 확인되지 않았고 손해는 확인됐으므로 넣지 않는다.
//        **PC 를 유선으로 붙인 뒤 다시 재서 판단할 것** — 지금 숫자는 Wi-Fi 위라
//        A/B 를 가릴 만큼 안정적이지 않다.
//
//  4) CAP_PROP_BUFFERSIZE 는 **쓰지 않는다 (쓸 수가 없다).**
//     FFmpeg 백엔드에서 `set(CAP_PROP_BUFFERSIZE, 1)` 이 **false 를 돌려주고**
//     `get()` 은 0 이다 (2026-08-04, 정순·역순 모두). 이 백엔드가 지원하지 않는
//     속성이라 호출해봐야 "버퍼를 줄였다"는 착각만 남는다.
//     우리가 실제로 통제할 수 있는 버퍼는 **아래 run() 의 백프레셔 큐** 하나다.
void video_worker::applyFfmpegCaptureOptions()
{
    // OpenCV 빌드 정보에서 avformat 버전을 뽑는다. 예: "avformat: YES (58.76.100)"
    static const QRegularExpression re(QStringLiteral("avformat:\\s*YES\\s*\\((\\d+)\\."));
    const QString info = QString::fromStdString(cv::getBuildInformation());
    const auto m = re.match(info);
    const int avformatMajor = m.hasMatch() ? m.captured(1).toInt() : 0;

    // 못 읽으면 옛 이름으로 간다 — 우리가 쓰는 번들(4.12/4.13)이 아직 FFmpeg 4.x 다.
    const bool newName = (avformatMajor >= 59);
    const QString timeoutOpt = newName ? QStringLiteral("timeout;5000000")
                                       : QStringLiteral("stimeout;5000000");

    // ⚠️ fflags;nobuffer 를 여기 다시 넣지 말 것 — 위 3) 참고.
    //
    //  5) analyzeduration / probesize — **채널 전환 체감의 핵심이다.**
    //     open 시간의 대부분은 avformat_find_stream_info() 의 탐색이다. 기본값이
    //     analyzeduration 5초 / probesize 5MB 로 넉넉한데, 우리는 스트림 구성을
    //     이미 안다(H.264 1080p + ONVIF 메타데이터 트랙). 그렇게 오래 볼 필요가 없다.
    //
    //     실측(2026-08-04 유선, 정순·역순 모두 재현):
    //         기본                    open 2591 / 2578 ms · 밀림 23 / 22 장
    //         analyzeduration 1s      open 1953 / 1957 ms · 밀림 12 / 11 장
    //         0.5s + probesize 500k   open 1447 / 1486 ms · 밀림  4 /  5 장
    //         **0.2s + probesize 200k** open 1141 / 1153 ms · 밀림  0 /  1 장  ← 채택
    //         0.1s + probesize 100k   open 1048 /  928 ms · 밀림  1 /  0 장
    //     모두 1920x1080 을 정상 인식했다. 0.1s 도 됐지만 여유를 두려고 0.2s 를 쓴다.
    //
    //     "밀림"이 왜 중요한가: open 하는 동안에도 카메라는 계속 보낸다. 그래서 열자마자
    //     **쌓여 있던 과거 프레임을 3~5ms 간격으로 몰아서** 받는다 — 화면은 1.5초 분량을
    //     되감기 재생하듯 훑고 나서야 실시간이 된다. 사용자가 "채널 들어갔다 나오면 렉
    //     걸린다"고 한 증상이 이것이다. 탐색을 줄이면 밀림 자체가 사라진다.
    //
    // ⚠️ 카메라 해상도/코덱을 바꾸면 이 값들의 여유가 달라진다. 영상이 안 뜨거나
    //    해상도를 잘못 잡으면 **여기부터** 의심할 것 (analyzeduration 을 늘려본다).
    const QString opts =
        QStringLiteral("rtsp_transport;tcp|%1|flags;low_delay"
                       "|analyzeduration;200000|probesize;200000")
            .arg(timeoutOpt);
    qputenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", opts.toUtf8());

    qDebug().noquote() << "[RTSP] avformat" << (avformatMajor ? QString::number(avformatMajor)
                                                              : QStringLiteral("불명"))
                       << "→ 옵션:" << opts;
}

video_worker::video_worker(const QString& rtspUrl, QObject *parent)
    : QThread(parent), m_rtspUrl(rtspUrl), m_stopRequested(false) {
    // Custom 구조체를 Signal/Slot에서 사용하기 위해 등록
    qRegisterMetaType<StreamStats>();
    qRegisterMetaType<QList<int>>("QList<int>");
    qRegisterMetaType<QList<QPolygonF>>("QList<QPolygonF>");
}

// 상시 ArUco 검출 — 4x4/5x5/6x6 사전을 차례로 시도해 처음 잡힌 것으로 고정.
// 오래 안 잡히면 다시 탐색 모드로 돌아간다.
//
// 🔴 **축소 + ROI 추적**으로 돈다. 원본 그대로 걸면 감당이 안 된다.
//    실측(2026-08-04, 당시 해상도 2592x1520, 현장 마커 4개 = ID 0/1/15/49):
//
//      전체 1.00배   73.7ms   4개 검출     ← 당시 30fps 예산(33.3ms)의 2.2배. 불가능
//      전체 0.50배   27.0ms   4개 검출
//      ROI  0.50배    3.3ms   4개 검출     ← 채택
//
//    ※ 카메라 프로파일이 1920x1080/15fps 로 바뀌어(2026-08-04) 픽셀이 47% 줄고
//      프레임 예산이 66.7ms 로 늘었다. 위 숫자는 **더 불리한 조건**의 값이므로
//      결론(ROI+0.5배 채택)은 그대로 유효하다. 다시 잴 필요는 없다.
//
//    ROI 는 직전에 찾은 마커들을 감싸는 사각형 + 여유다(전체의 10% 남짓).
//    놓치면 그 프레임만 전체 재탐색하고 다시 좁힌다.
//
// ⚠️ DetectorParameters 는 **기본값을 쓴다.** 튜닝(임계 1단계 + minMarkerPerimeterRate
//    상향)이 1.9ms 로 제일 빨랐지만 **마커 4개 중 3개를 놓쳤다** — ROI 안에서는 마커가
//    상대적으로 커져서 maxMarkerPerimeterRate 에 걸린다. 빨라도 못 찾으면 의미가 없다.
//
// ⚠️ 축소 배율은 0.5 를 쓴다. 정확히 1/2 이라 OpenCV 가 빠른 경로를 타서 리사이즈가
//    0.07ms 로 공짜다. 0.4/0.33 은 검출이 더 빠른 대신 리사이즈에서 2.6ms 를 도로 뱉는다.
void video_worker::detectArucoAlways(const cv::Mat &bgr)
{
    using namespace cv::aruco;
    static const Dictionary dicts[] = {
        getPredefinedDictionary(DICT_4X4_50),
        getPredefinedDictionary(DICT_5X5_100),
        getPredefinedDictionary(DICT_6X6_250),
    };
    static const ArucoDetector detectors[] = {
        ArucoDetector(dicts[0], DetectorParameters()),
        ArucoDetector(dicts[1], DetectorParameters()),
        ArucoDetector(dicts[2], DetectorParameters()),
    };
    constexpr int kDictCount = 3;
    constexpr double kScale = 0.5;          // 축소 배율 (1/2 = 빠른 경로)
    constexpr double kInv   = 1.0 / kScale;

    // 그레이 변환은 여기서 한 번만. detectMarkers 는 컬러를 받으면 내부에서 매번
    // 그레이로 바꾸므로, 사전 3종을 도는 탐색 모드에서는 그 변환도 3번 돌았다.
    cv::Mat gray, small;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, small, cv::Size(), kScale, kScale, cv::INTER_AREA);

    // 축소본 기준 ROI. 비어 있으면 전체.
    cv::Rect roi(0, 0, small.cols, small.rows);
    bool usingRoi = false;
    if (m_arucoRoiValid) {
        const cv::Rect r = m_arucoRoi & roi;   // 프레임 밖으로 나간 부분은 잘라낸다
        if (r.width > 40 && r.height > 40) { roi = r; usingRoi = true; }
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    auto runDetect = [&](int dict, const cv::Rect &r) {
        detectors[dict].detectMarkers(small(r), corners, ids);
        // ROI 로컬 좌표 → 축소본 전체 좌표
        if (r.x || r.y)
            for (auto &m : corners) for (auto &p : m) { p.x += r.x; p.y += r.y; }
    };

    if (m_arucoDictLock >= 0) {
        runDetect(m_arucoDictLock, roi);
        // ROI 에서 놓쳤으면 그 프레임만 전체로 다시 본다. 로봇이 빠르게 움직여
        // ROI 를 벗어나는 경우가 여기서 회복된다.
        if (ids.empty() && usingRoi)
            runDetect(m_arucoDictLock, cv::Rect(0, 0, small.cols, small.rows));
        if (ids.empty()) {
            if (++m_arucoMiss > 40) { m_arucoDictLock = -1; m_arucoMiss = 0; }
        } else {
            m_arucoMiss = 0;
        }
    } else {
        // ⚠️ 사전을 못 잡은 상태에서는 3개를 전부 돌린다. 축소 전에는 3종 전탐색이
        //    170ms 였다(1080p 실측). 보드 마커를 치우는 게 최종 목표라 그 뒤에는 이
        //    상태가 **영구히** 지속된다 — 아무것도 못 찾는 탐색에 코어를 계속 태우게
        //    된다. 그래서 실패가 쌓일수록 탐색 간격을 늘린다.
        //    ⚠️ 탐색은 항상 **전체 프레임**이다. ROI 는 직전 검출이 있어야 의미가 있다.
        if (m_searchSkip > 0) { --m_searchSkip; return; }
        const cv::Rect full(0, 0, small.cols, small.rows);
        for (int d = 0; d < kDictCount; ++d) {
            runDetect(d, full);
            if (!ids.empty()) { m_arucoDictLock = d; break; }
        }
        m_searchFail = ids.empty() ? qMin(m_searchFail + 1, 12) : 0;
        m_searchSkip = m_searchFail;
    }

    // 다음 프레임용 ROI 갱신 — 찾은 마커 전체를 감싸고 여유를 준다.
    // 여유는 프레임 사이에 마커가 움직일 수 있는 거리 + 검출 흔들림을 덮을 정도.
    if (ids.empty()) {
        m_arucoRoiValid = false;            // 놓쳤으면 다음엔 전체부터
    } else {
        float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
        for (const auto &m : corners) for (const auto &p : m) {
            x0 = std::min(x0, p.x); y0 = std::min(y0, p.y);
            x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
        }
        const float pad = float(small.cols) * 0.06f + 40.0f;   // 해상도에 비례 + 하한
        m_arucoRoi = cv::Rect(int(x0 - pad), int(y0 - pad),
                              int((x1 - x0) + 2 * pad), int((y1 - y0) + 2 * pad));
        m_arucoRoiValid = true;
    }

    // 빈 결과는 상태가 바뀔 때 한 번만 보낸다 (오버레이 지우기용)
    if (ids.empty() && m_lastArucoCount == 0) return;
    m_lastArucoCount = int(ids.size());

    // 🔴 코너를 **원본 프레임 좌표로 되돌린다.** 검출은 축소본에서 했지만, 이 좌표를
    //    받는 쪽(VideoView 오버레이·TopView H 변환)은 전부 원본 픽셀 기준이다.
    //    되돌리지 않으면 마커 표시가 화면 좌상단 1/4 에 몰려 찍힌다.
    //    실측 검증(1080p): 원본 검출과 축소×2 의 코너 차이 최대 2.24px (한 변 76px).
    QList<int> qids;
    QList<QPolygonF> qpolys;
    for (size_t i = 0; i < ids.size(); ++i) {
        qids.append(ids[i]);
        QPolygonF poly;
        for (const cv::Point2f &p : corners[i])
            poly << QPointF(p.x * kInv, p.y * kInv);
        qpolys.append(poly);
    }
    emit arucoDetected(qids, qpolys);
}

video_worker::~video_worker() {
    stop();
    wait();
}

void video_worker::stop() {
    m_stopRequested = true;
}

// stop() 이 걸린 뒤에도 최대 ms 만큼 기다리되, 100ms 마다 중단 요청을 확인한다.
// 재연결 백오프 중에 종료를 몇 초씩 못 듣는 일이 없도록.
bool video_worker::sleepInterruptible(int ms)
{
    for (int slept = 0; slept < ms; slept += 100) {
        if (m_stopRequested) return false;
        msleep(100);
    }
    return !m_stopRequested;
}

void video_worker::run() {
    StreamStats stats;
    const std::string url = m_rtspUrl.toStdString();

    // ⚠️ RTSP 캡처 옵션(OPENCV_FFMPEG_CAPTURE_OPTIONS)은 여기서 세팅하지 않는다.
    //    qputenv 는 **프로세스 전역**인데 워커가 여러 개(본 화면 1 + 미리보기 4)라
    //    서로 덮어쓰게 된다. main() 에서 딱 한 번 설정한다 —
    //    video_worker::applyFfmpegCaptureOptions() 참고.

    cv::VideoCapture cap;
    // ⚠️ 프레임 버퍼는 여기 하나로 두지 않는다 — 루프 안에서 nextFrameBuffer() 로
    //    풀에서 받는다. 하나만 돌려쓰면 화면에 떠 있는 버퍼를 다음 프레임이 덮어쓴다.

    bool everGotFrame = false;  // 한 번이라도 받았다 = 주소·계정이 맞다는 증거
    int  retry = 0;

    // FPS 및 Latency 측정을 위한 타이머 변수들
    QElapsedTimer fpsTimer;
    fpsTimer.start();
    int frameCount = 0;

    double emaDecodeMs = 0.0;   // 지수이동평균 — 한 프레임 튄다고 표시가 요동치지 않게

    while (!m_stopRequested) {
        // 0. 연결이 없으면 (재)연결한다.
        // ⚠️ 스트림은 열린 뒤에도 죽는다 — 카메라 인코더 재시작, 세션 경합, 네트워크.
        //    예전에는 죽은 핸들에 grab() 을 영원히 다시 걸기만 해서 한 번 끊기면
        //    되살아나지 못했다. FPS 가 0.1 에 고정되던 증상이 이것이다.
        if (!cap.isOpened()) {
            if (retry > 0 && !sleepInterruptible(qMin(1000 << (retry - 1), 15000)))
                break;
            if (!cap.open(url, cv::CAP_FFMPEG)) {
                stats.isConnected = false;
                stats.fps = 0.0;
                emit statsUpdated(stats);
                // ⚠️ 처음부터 못 열면 주소나 계정이 틀린 것이다. 계속 두드리면 카메라가
                //    계정을 잠근다 (Hanwha: RTSP/1.0 490 Account Blocked) → 한 번만
                //    시도하고 포기한다. 반대로 한 번이라도 프레임을 받아본 뒤라면
                //    자격증명은 확실히 맞으므로, 잠길 걱정 없이 재연결해도 된다.
                if (!everGotFrame) {
                    qDebug() << "RTSP 스트림을 열 수 없습니다:" << m_rtspUrl;
                    emit openFailed(m_rtspUrl);
                    return;
                }
                ++retry;
                continue;
            }
            // ⚠️ 여기서 retry 를 0 으로 되돌리면 안 된다. 카메라가 접속은 받아주면서
            //    데이터를 거의 안 보내는 상태에서는 "열기 성공 → 프레임 실패 → 재접속"
            //    이 1초마다 무한 반복되고 백오프가 전혀 자라지 않는다. 매 반복이 새
            //    RTSP 세션이라 카메라에 세션이 쌓여 상태를 더 나쁘게 만든다.
            //    retry 는 **프레임을 실제로 받았을 때만** 0 으로 돌린다.
            stats.isConnected = true;
            emit statsUpdated(stats);
            if (everGotFrame) emit streamReconnected();
            fpsTimer.restart();
            frameCount = 0;
        }

        // 1. 프레임 읽기 시도 (grab / retrieve 분리형으로 통일)
        // ⚠️ grab() 은 **다음 프레임이 도착할 때까지 블로킹**한다. 그래서 grab 을 포함해
        //    시간을 재면 프레임 간격이 통째로 섞여 들어온다 (15fps 면 66.7ms).
        //    그건 지연이 아니라 그냥 기다린 시간이다 → retrieve(디코드)만 잰다.
        if (!cap.grab()) {
            stats.frameDropCount++;
            stats.isConnected = false;
            stats.fps = 0.0;            // 끊긴 채로 옛 FPS 를 계속 보여주면 안 된다
            emit statsUpdated(stats);
            cap.release();              // 죽은 핸들을 버리고 위에서 다시 연다
            ++retry;
            continue;
        }

        stats.isConnected = true; // grab 성공 시 연결 유지로 판단

        // 🔴 디코더가 쓸 버퍼를 **풀에서** 받는다. 화면이 아직 물고 있는 슬롯은
        //    nextFrameBuffer() 가 알아서 피한다. 이게 있어야 matToQImage 에서
        //    깊은 복사를 없앨 수 있다 (2592x1520 에서 프레임당 3.11ms).
        cv::Mat &frame = nextFrameBuffer();

        const int64 tDecode = cv::getTickCount();
        if (!cap.retrieve(frame) || frame.empty()) {
            stats.frameDropCount++;
            emit statsUpdated(stats);
            continue;
        }
        everGotFrame = true;
        retry = 0;              // 프레임이 실제로 왔을 때만 백오프를 푼다
        const double decodeMs =
            ((cv::getTickCount() - tDecode) * 1000.0) / cv::getTickFrequency();

        // 2. 디코드 시간 (표시용). I-프레임은 P-프레임보다 훨씬 크고 느려서 값이 오르내린다.
        //    평균을 보여줘야 조작자가 "지금 밀리는 중"인지 판단할 수 있다.
        emaDecodeMs = (emaDecodeMs <= 0.0) ? decodeMs : (emaDecodeMs * 0.85 + decodeMs * 0.15);
        stats.latencyMs = emaDecodeMs;

        // ⚠️ 예전에는 여기서 "60ms 넘으면 cap.grab() 한 번 더 해서 버린다" 를 했다.
        //    그 값에는 프레임 간격이 섞여 있어서 거의 항상 걸렸고(당시 30fps=33ms),
        //    당시 GOV 60 이라 2초마다 오는 I-프레임에서 확실히 걸렸다. 걸리면 프레임을
        //    버려 FPS 가 떨어지고, 버린 만큼 다음 grab 이 더 오래 걸려 **또** 걸리는
        //    자기증폭 루프가 됐다. 화면의 FPS·지연이 2초 주기로 튄 원인이 이것이다.
        //    아래 루프는 오는 대로 즉시 소비하므로 버퍼가 쌓이지 않는다 — 버리지 않는다.
        //    🔴 **"따라잡기(grab-and-drop)"를 여기 다시 넣지 말 것 — 재서 확인했다.**
        //       grab() 블로킹 시간을 기준으로 버리는(자기증폭 없는) 안전한 방식으로
        //       구현해 2회차 교차 측정한 결과(2026-08-04, 유선):
        //           기준(read)        15.0 fps · 버림 0 · drift +8 / +2 ms
        //           따라잡기 2ms/2장  15.0 fps · **버림 0** · drift -3 / +5 ms
        //           따라잡기 5ms/5장  15.0 fps · **버림 0** · drift +16 / -4 ms
        //       **버린 프레임이 0장이다 — 버릴 게 없다.** grab() 이 매번 약 59ms
        //       (한 프레임 간격)를 기다린다 = 이미 라이브 끝단이라 밀린 프레임이
        //       존재하지 않는다. 이득 0 인 기법을, 그것도 예전에 자기증폭 루프를
        //       만들었던 기법을 들일 이유가 없다.
        //       ※ Wi-Fi 로 붙었을 때는 이득이 있어 보였는데, 그건 네트워크가 만든
        //         밀림을 걷어내던 것이고 회차마다 결론이 뒤집혔다. 유선이 정답이었다.

        // 3. FPS 계산 (1초마다 갱신)
        frameCount++;
        if (fpsTimer.elapsed() >= 1000) {
            stats.fps = (frameCount * 1000.0) / fpsTimer.elapsed();
            frameCount = 0;
            fpsTimer.restart();

            // 1초 주기로 UI에 통계 데이터 전송 (잦은 UI 갱신으로 인한 부하 방지)
            emit statsUpdated(stats);
        }


        // --- [영상 변형 시작] ---
        m_mutex.lock();
        int b = m_brightness;
        int c = m_contrast;
        int s = m_sharpen;
        int sat = m_saturation;
        m_mutex.unlock();

        // ⚠️ 여기서 프레임을 펴지 않는다. 캡처 프레임은 **카메라 원본(왜곡 상태)** 이
        //    그대로 나가고, 왜곡 보정은 좌표 단계에서만 한다:
        //      · TopView  — VideoView 가 camcalib 로 점을 펴서 H 적용
        //      · RTSP 뷰  — 도면 점을 정변환(distort)해서 왜곡 화면에 맞춰 그림
        //    ArUco 검출 좌표도 원본 픽셀이므로 위 두 경로와 좌표계가 일치한다.

        // 1. 밝기 및 대비 조절
        // 기본값(0/0)이면 alpha=1, beta=0 이라 결과가 원본과 같다. 그런데도 convertTo 는
        // 1920x1080x3 을 통째로 곱하고 새 버퍼에 쓴다 — 매 프레임 낭비다. 건너뛴다.
        if (b != 0 || c != 0) {
            double alpha = (c / 100.0) + 1.0;
            frame.convertTo(frame, -1, alpha, b);
        }

        // 2. 채도 조절
        // ⚠️ 예전에는 at<Vec3b>() 로 200만 픽셀을 직접 돌았다 — 1080p 에서 20~40ms 라
        //    채도를 0 이 아닌 값으로 한 번만 만져두면 캡처 스레드가 그대로 반토막 났다.
        //    split/convertTo 는 같은 일을 SIMD 로 한다.
        if (sat != 0) {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> ch;
            cv::split(hsv, ch);
            ch[1].convertTo(ch[1], -1, 1.0, sat);   // 포화 처리는 convertTo 가 알아서 한다
            cv::merge(ch, hsv);
            cv::cvtColor(hsv, frame, cv::COLOR_HSV2BGR);
        }

        // 3. 선명도 조절
        if (s > 0) {
            cv::Mat blurred;
            cv::GaussianBlur(frame, blurred, cv::Size(9, 9), s / 10.0);
            cv::addWeighted(frame, 1.5, blurred, -0.5, 0, frame);
        }
        // --- [영상 변형 끝] ---

        // 상시 ArUco 검출 (5프레임마다 1회 — 부하 억제). 프레임에는 그리지 않는다.
        //
        // 🔴 꺼져 있으면 **검출 자체를 건너뛴다.** 예전에는 `ArUco OFF` 가 화면 표시만
        //    숨기고 검출은 계속 돌았다 — 끄나 켜나 CPU 는 똑같이 태웠다.
        //    이 검출은 **화면 표시 전용**이다. 로봇의 실제 위치는 CCTV 앱이 따로 검출해
        //    서버로 보내고, 서버가 POSE 로 내려준다(별개 파이프라인). 즉 여기서 하는 건
        //    같은 일을 한 번 더 하는 것이라, 필요 없을 때 끄면 그만큼이 그대로 남는다.
        if (m_arucoEnabled.load(std::memory_order_relaxed)) {
            // 5프레임마다 → **2프레임마다**. 주기는 프레임 수가 아니라 시간으로 봐야 한다:
            // 30fps 시절 5프레임 = 167ms 였지만, 카메라가 15fps 로 바뀌면서 같은 5프레임이
            // 333ms 가 됐다 — 오버레이가 눈에 띄게 늦는다. 2프레임이면 133ms 로 돌아온다.
            // 비용은 감당된다: 검출이 ROI 0.5배에서 3.3ms(2592x1520 실측, 1080p 는 더 싸다)
            // 인데 프레임 예산이 66.7ms 다.
            if (++m_frameCounter % 2 == 0)
                detectArucoAlways(frame);
        } else if (m_lastArucoCount != 0) {
            // 끄는 순간 화면에 남아 있던 오버레이를 한 번 지운다
            m_lastArucoCount = 0;
            m_arucoRoiValid = false;
            emit arucoDetected(QList<int>(), QList<QPolygonF>());
        }

        // ⚠️ 여기 있던 영상 필터 분기(고대비 흑백/컬러, 상하반전, TopView 워프,
        //    ArUco 그리기)는 **전부 죽은 코드**였다 — setFilterMode() 를 어디서도
        //    호출하지 않아 m_filterMode 는 항상 Original 이었다. TopView 는
        //    VideoView 가 따로 그리고, ArUco 는 위의 detectArucoAlways() 가 한다.
        //    지우면서 Original 이 아닐 때의 프레임 전체 복사도 같이 사라졌다.
        cv::Mat &processedFrame = frame;

        // 상단 데이터 및 자막 표시 텍스트 그리기 (필터 처리된 영상 위에 오버레이)
        // std::string text = "FPS: " + std::to_string(stats.fps).substr(0, 4) +"  Latency: " + std::to_string(stats.latencyMs).substr(0, 4) + "ms";
        // cv::putText(processedFrame, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (!stats.isConnected) {
            cv::putText(processedFrame, "DISCONNECTED", cv::Point(20, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2); // 적색
        }

        // 4. 최종 결과 영상 변환 및 UI 전송 (반드시 연산이 다 끝난 processedFrame을 전송!)
        // ⚠️ GUI 가 못 따라가면 버린다. 큐드 연결이라 밀어넣는 건 언제나 성공하지만,
        //    쌓이면 화면은 과거를 보여주고 메모리는 계속 는다. 실시간 감시 화면에서
        //    "늦은 프레임"은 가치가 없으므로 최신 것만 살린다.
        // 🔴 **이것이 우리가 실제로 줄일 수 있는 유일한 버퍼다.**
        //    CAP_PROP_BUFFERSIZE 는 FFmpeg 백엔드가 아예 안 받는다(위 4번 참고).
        //
        //    깊이 3 → 2 → **1**. 한 칸의 값어치가 프레임 간격이라, 카메라가
        //    30fps 일 때는 한 칸이 33.3ms 였지만 **15fps 로 바뀌면서 66.7ms** 가 됐다.
        //    깊이 2 면 최대 133ms 를 화면이 과거로 사는 셈이라, 같은 깊이를 두면
        //    프로파일 변경만으로 지연이 두 배가 된다. 그래서 1 로 줄인다.
        //    ("첫 화면은 느려도 되고 그 뒤 실시간성이 중요"라는 결정과 같은 방향)
        //
        // ⚠️ 1 은 "GUI 가 직전 프레임을 아직 안 가져갔으면 이번 건 버린다"는 뜻이다.
        //    버려도 안전한 이유: 이 프레임은 **표시 전용**이고, 로봇 위치는 서버
        //    POSE 로 따로 온다. 그리고 GUI 쪽 실측 부담이 프레임당 3.4ms 라
        //    66.7ms 예산에서 큐가 찰 일 자체가 드물다 — 찬다면 그건 GUI 가 진짜로
        //    막힌 상황이고, 그때는 옛 프레임을 쌓는 것보다 버리는 게 맞다.
        if (m_queued.load(std::memory_order_relaxed) < 1) {
            m_queued.fetch_add(1, std::memory_order_relaxed);
            QImage qimg = matToQImage(processedFrame);
            emit frameReceived(qimg);
        } else {
            stats.frameDropCount++;
        }

        // RTSP의 자체 스트림 속도에 맞추되, CPU 과점유 방지용 미세 대기
        msleep(1);
    }

    cap.release();
    stats.isConnected = false;
    emit statsUpdated(stats);
}

// ⚠️ **BGR888 로 내보낸다.** OpenCV 는 BGR, Qt 는 RGB 라 예전엔 프레임마다
//    cvtColor(BGR→RGB) 를 돌렸는데, 2592x1520x3 = 11.8MB 를 한 번 더 훑는 일이다.
//    Qt 5.14 부터 QImage::Format_BGR888 이 있어서 **변환 없이 그대로 넘길 수 있다.**
//
//    실측(2026-07-31, 1920x1080). 워커/GUI 를 **합쳐서** 봐야 한다 — 한쪽만 보면
//    스레드 사이로 비용을 옮기고 개선했다고 착각한다:
//        A RGB888(예전)   워커 2.93 + GUI 1.82 = 4.75ms
//        B BGR888(지금)   워커 1.43 + GUI 1.94 = 3.37ms   ← 채택
//        C RGB32(네이티브) 워커 3.44 + GUI 0.50 = 3.94ms
//    B 는 합계도 최소고, **병목인 워커 스레드**에서 부담을 덜어낸다.
//
// ⚠️ 이 포맷을 바꾸면 받는 쪽도 같이 맞춰야 한다. 안 맞추면 변환이 사라지는 게 아니라
//    GUI 스레드로 옮겨갈 뿐이다. 지금 맞춰 둔 곳:
//      · VideoView::paint 의 drawImage — Qt 가 BGR888 을 그대로 그린다 (수정 불필요)
//      · VideoView::warpToTopView — 입출력 포맷을 BGR888 로 맞춰 뒀다
//      · channel_tile — 미리보기 타일도 같은 포맷으로 받는다
// 다음 프레임을 받을 버퍼. 아직 화면에서 쓰는 중인 슬롯은 건드리지 않는다.
// (자세한 이유는 video_worker.h 의 프레임 풀 주석 참고)
cv::Mat &video_worker::nextFrameBuffer()
{
    m_poolIdx = (m_poolIdx + 1) % int(m_pool.size());
    cv::Mat &s = m_pool[m_poolIdx];
    // refcount > 1 = QImage 가 아직 이 버퍼를 물고 있다 → 덮어쓰면 화면이 깨진다.
    // 빈 Mat 을 넣어두면 다음 retrieve() 가 새 버퍼를 잡는다.
    if (!s.empty() && s.u && s.u->refcount > 1) s = cv::Mat();
    return s;
}

// QImage 가 죽을 때 물고 있던 Mat 사본을 놓아준다 → 그때서야 버퍼가 해제된다.
static void releasePooledMat(void *info) { delete static_cast<cv::Mat *>(info); }

QImage video_worker::matToQImage(const cv::Mat& mat) {
    if (mat.type() == CV_8UC3) {
        // 🔴 **복사하지 않는다.** 얕은 복사(refcount++)한 Mat 을 QImage 의 cleanup 에
        //    물려서, QImage 가 살아 있는 동안 픽셀 버퍼가 유지되게 한다.
        //    깊은 복사(.copy())는 2592x1520 에서 3.11ms 였다 (2026-08-04 실측).
        //    ⚠️ 이게 성립하려면 **호출부가 nextFrameBuffer() 로 받은 버퍼를 넘겨야 한다.**
        //       아무 Mat 이나 넘기면 그 Mat 이 재사용될 때 화면이 깨진다.
        auto *hold = new cv::Mat(mat);
        return QImage(hold->data, hold->cols, hold->rows, hold->step,
                      QImage::Format_BGR888, releasePooledMat, hold);
    }
    else if (mat.type() == CV_8UC1) { // 8-bit, 1채널 (Grayscale)
        return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
    }
    return QImage();
}
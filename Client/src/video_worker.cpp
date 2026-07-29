#include "video_worker.h"
#include <QDebug>
#include <opencv2/objdetect/aruco_detector.hpp>

video_worker::video_worker(const QString& rtspUrl, QObject *parent)
    : QThread(parent), m_rtspUrl(rtspUrl), m_stopRequested(false) {
    // Custom 구조체를 Signal/Slot에서 사용하기 위해 등록
    qRegisterMetaType<StreamStats>();
    qRegisterMetaType<QList<int>>("QList<int>");
    qRegisterMetaType<QList<QPolygonF>>("QList<QPolygonF>");
}

// 상시 ArUco 검출 — 4x4/5x5/6x6 사전을 차례로 시도해 처음 잡힌 것으로 고정.
// 오래 안 잡히면 다시 탐색 모드로 돌아간다.
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

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    if (m_arucoDictLock >= 0) {
        detectors[m_arucoDictLock].detectMarkers(bgr, corners, ids);
        if (ids.empty()) {
            if (++m_arucoMiss > 40) { m_arucoDictLock = -1; m_arucoMiss = 0; }
        } else {
            m_arucoMiss = 0;
        }
    } else {
        // ⚠️ 사전을 못 잡은 상태에서는 3개를 전부 돌린다. 1080p 에서 검출 한 번이
        //    20~30ms 라 3배면 100ms 에 육박한다. 그리고 보드 마커를 치우는 게 최종
        //    목표이므로, 그 뒤에는 이 상태가 **영구히** 지속된다 — 아무것도 못 찾는
        //    탐색에 코어 절반을 계속 태우게 된다. 그래서 실패가 쌓일수록 탐색 간격을
        //    늘린다 (이 함수 자체가 5프레임마다 불리므로, 최대 12회 건너뛰면 ≈2초).
        if (m_searchSkip > 0) { --m_searchSkip; return; }
        for (int d = 0; d < kDictCount; ++d) {
            detectors[d].detectMarkers(bgr, corners, ids);
            if (!ids.empty()) { m_arucoDictLock = d; break; }
        }
        m_searchFail = ids.empty() ? qMin(m_searchFail + 1, 12) : 0;
        m_searchSkip = m_searchFail;
    }

    // 빈 결과는 상태가 바뀔 때 한 번만 보낸다 (오버레이 지우기용)
    if (ids.empty() && m_lastArucoCount == 0) return;
    m_lastArucoCount = int(ids.size());

    QList<int> qids;
    QList<QPolygonF> qpolys;
    for (size_t i = 0; i < ids.size(); ++i) {
        qids.append(ids[i]);
        QPolygonF poly;
        for (const cv::Point2f &p : corners[i])
            poly << QPointF(p.x, p.y);
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

    // RTP 를 TCP 로 강제하고, 소켓 읽기 타임아웃을 5초로 줄인다.
    //
    // - 기본값(UDP)은 이 현장에서 30초쯤 타임아웃을 먹고서야 붙는다 — 같은 카메라가
    //   TCP 로는 4초면 첫 프레임이 나온다.
    // - timeout 을 안 주면 FFmpeg 기본이 30초다. 카메라 쪽에서 스트림이 끊기면
    //   (다른 작업자가 웹 UI 에서 프로파일을 저장하면 해당 인코더가 재시작되면서
    //   붙어 있던 RTSP 세션이 전부 끊긴다) grab() 하나가 30초를 통째로 잡아먹고,
    //   그 30초 동안은 stop() 도 못 듣는다 → 주소를 바꾸면 옛 워커가 살아남아
    //   디코더가 두 개로 겹친다. 로그에 h264 컨텍스트가 두 개 찍히던 원인.
    //
    // OpenCV 는 이 환경변수를 VideoCapture 를 열 때 읽으므로 반드시 생성 **전에**
    // 세팅해야 한다.
    //
    // ⚠️ 소켓 타임아웃 옵션 이름은 반드시 `stimeout` 이다. FFmpeg 의 RTSP 디먹서에서
    //    `timeout` 은 뜻이 전혀 다르다 — "들어오는 연결을 기다리는 시간"이고, 주는
    //    순간 listen 플래그가 켜져서 클라이언트가 아니라 **서버**로 동작하려 든다.
    //    카메라에 접속도 못 해보고 이렇게 죽는다:
    //        [rtsp @ ...] Unable to open RTSP for listening
    //    둘 다 넣으면 안 된다. 하나는 무시되는 게 아니라 다른 뜻으로 먹힌다.
    qputenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "rtsp_transport;tcp|stimeout;5000000");

    cv::VideoCapture cap;
    cv::Mat frame;

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
        //    시간을 재면 30fps 스트림에서는 프레임 간격 33ms 가 통째로 섞여 들어온다.
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
        //    그 값에는 프레임 간격이 섞여 있어서 30fps(33ms)에서는 거의 항상 걸렸고,
        //    GOV 60 이라 2초마다 오는 I-프레임에서 확실히 걸렸다. 걸리면 프레임을 버려
        //    FPS 가 떨어지고, 버린 만큼 다음 grab 이 더 오래 걸려 **또** 걸리는
        //    자기증폭 루프가 됐다. 화면의 FPS·지연이 2초 주기로 튄 원인이 이것이다.
        //    아래 루프는 오는 대로 즉시 소비하므로 버퍼가 쌓이지 않는다 — 버리지 않는다.

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
        if (++m_frameCounter % 5 == 0)
            detectArucoAlways(frame);

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
        if (m_queued.load(std::memory_order_relaxed) < 3) {
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

QImage video_worker::matToQImage(const cv::Mat& mat) {
    if (mat.type() == CV_8UC3) { // 8-bit, 3채널 (BGR)
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        return QImage((const unsigned char*)(rgbMat.data), rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888).copy();
    }
    else if (mat.type() == CV_8UC1) { // 8-bit, 1채널 (Grayscale)
        return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
    }
    return QImage();
}
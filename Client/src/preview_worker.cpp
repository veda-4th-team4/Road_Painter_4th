#include "preview_worker.h"
#include "videofilters.h"

#include <QDebug>
#include <opencv2/opencv.hpp>

namespace {
// 미리보기 타일에 떠 있어도 되는 프레임 수. 워커가 4개라 이 값 x4 가 큐에 뜬다.
// ⚠️ 저해상도 서브 프로파일이 없어졌다(2026-08-04) — 미리보기도 메인과 같은
//    2592x1520 을 받는다. BGR 한 장이 ~11.8MB이므로 4채널 최신 프레임만
//    유지해도 약 47MB다. 20fps의 늦은 프레임은 감시 화면에서 가치가
//    없으므로 video_worker 와 같이 1 로 줄인다 (= 최신 것만 살린다).
constexpr int kMaxQueued = 1;

// ⚠️ video_worker::matToQImage 와 **같은 포맷(BGR888)** 이어야 한다. 미리보기 타일과
//    본 화면이 다른 포맷을 쓰면 한쪽만 빨강↔파랑이 뒤집힌다.
//    cvtColor 를 안 타므로 워커 4개가 각자 한 번씩 아끼는 셈이다.
QImage matToQImage(const cv::Mat &mat)
{
    if (mat.type() == CV_8UC3) {   // BGR — 변환 없이 그대로 감싼다
        return QImage(reinterpret_cast<const uchar *>(mat.data), mat.cols, mat.rows,
                      mat.step, QImage::Format_BGR888).copy();
    }
    if (mat.type() == CV_8UC1) {
        return QImage(reinterpret_cast<const uchar *>(mat.data), mat.cols, mat.rows,
                      mat.step, QImage::Format_Grayscale8).copy();
    }
    return QImage();
}
} // namespace

preview_worker::preview_worker(int ch, const QString &rtspUrl, QObject *parent)
    : QThread(parent), m_ch(ch), m_rtspUrl(rtspUrl)
{
}

preview_worker::~preview_worker()
{
    stop();
    // 🔴 대기 시간은 **RTSP 소켓 타임아웃(stimeout)보다 길어야 한다.**
    //    캡처 스레드는 cap.grab() 안에서 최대 그만큼 막혀 있을 수 있는데, 그전에
    //    wait 가 풀리면 QThread 기반 클래스가 **아직 도는 스레드를 파괴**하게 되고
    //    "QThread: Destroyed while thread is still running" 으로 죽는다.
    //    예전 값이 3000 이었다 — stimeout 이 5초라 카메라가 죽은 채로 앱을 닫으면
    //    타임아웃이 나서 그대로 크래시였다.
    // ⚠️ video_worker.cpp 의 stimeout 값을 바꾸면 여기도 같이 올려야 한다.
    if (!wait(7000))
        qWarning() << "[preview] CH" << m_ch << "캡처 스레드가 7초 안에 안 끝났습니다";
}

void preview_worker::setVideoFilters(int brightness, int contrast, int sharpen, int saturation)
{
    m_brightness.store(qBound(-100, brightness, 100), std::memory_order_relaxed);
    m_contrast.store(qBound(-100, contrast, 100), std::memory_order_relaxed);
    m_sharpen.store(qBound(0, sharpen, 100), std::memory_order_relaxed);
    m_saturation.store(qBound(-100, saturation, 100), std::memory_order_relaxed);
}

void preview_worker::stop()
{
    m_stopRequested.store(true, std::memory_order_relaxed);
}

bool preview_worker::sleepInterruptible(int ms)
{
    while (ms > 0 && !m_stopRequested.load(std::memory_order_relaxed)) {
        const int slice = qMin(ms, 100);
        msleep(slice);
        ms -= slice;
    }
    return !m_stopRequested.load(std::memory_order_relaxed);
}

void preview_worker::run()
{
    const std::string url = m_rtspUrl.toStdString();

    // 🔴 여기서 qputenv 를 **하지 않는다.** 예전에는 했는데, 그건 버그였다.
    //    qputenv 는 프로세스 전역이고 OpenCV 는 VideoCapture 를 **열 때** 읽는다.
    //    여기서 덮어쓰면 메인 워커가 열릴 때 이 값을 집어간다 — 예전엔 두 값이
    //    같아서 티가 안 났지만, 메인 쪽에 flags;low_delay 와 짧은 탐색(analyzeduration/
    //    probesize)이 붙으면서 **값이 달라졌다.** 그때부터는 "누가 마지막에 열었나"에
    //    따라 옵션이 달라지는 경합이 된다.
    //    옵션은 main() 에서 video_worker::applyFfmpegCaptureOptions() 로 딱 한 번
    //    설정한다 — 미리보기도 그 값을 그대로 쓰는 것이 맞다.

    cv::VideoCapture cap;
    cv::Mat frame;
    bool everGotFrame = false;   // 한 번이라도 받았다 = 주소·계정이 맞다는 증거
    int retry = 0;
    bool live = false;

    auto setLive = [&](bool v) {
        if (live == v) return;
        live = v;
        emit liveChanged(m_ch, v);
    };

    while (!m_stopRequested.load(std::memory_order_relaxed)) {
        if (!cap.isOpened()) {
            // 지수 백오프 (1s, 2s, 4s … 최대 15s). 중계는 on-demand 라 첫 프레임까지
            // 1~3초가 정상이므로 조급하게 다시 열지 않는다.
            if (retry > 0 && !sleepInterruptible(qMin(1000 << (retry - 1), 15000)))
                break;
            if (!cap.open(url, cv::CAP_FFMPEG)) {
                setLive(false);
                // 🔴 처음부터 못 열면 포기한다. 주소나 계정이 틀린 것이고, 계속
                //    두드리면 Hanwha 카메라가 계정을 잠근다. 미리보기는 워커가
                //    4개라 같은 실수의 대가가 4배다.
                if (!everGotFrame) {
                    qDebug() << "[preview] CH" << m_ch << "스트림을 열 수 없습니다:" << m_rtspUrl;
                    emit openFailed(m_ch, m_rtspUrl);
                    return;
                }
                ++retry;
                continue;
            }
            // ⚠️ 여기서 retry 를 0 으로 되돌리지 않는다. 접속은 받아주면서 데이터를
            //    안 보내는 상태에서 "열기 성공 → 프레임 실패 → 재접속"이 1초마다
            //    무한 반복되고 백오프가 자라지 않는다. retry 는 프레임을 실제로
            //    받았을 때만 푼다.
        }

        if (!cap.grab()) {
            setLive(false);
            cap.release();   // 죽은 핸들을 버리고 위에서 다시 연다
            ++retry;
            continue;
        }
        // 일시정지 중에는 여기서 끝낸다 — grab() 으로 패킷은 계속 받아 세션을 살려두되
        // 색변환·QImage·시그널은 건너뛴다. 그래야 그리드로 복귀할 때 다시 열지 않는다
        // (다시 열면 1.1초 + 4채널 직렬화로 4.6초가 든다. preview_worker.h 참고).
        if (m_paused.load(std::memory_order_relaxed)) {
            everGotFrame = true;
            retry = 0;
            continue;
        }

        if (!cap.retrieve(frame) || frame.empty())
            continue;

        everGotFrame = true;
        retry = 0;          // 프레임이 실제로 왔을 때만 백오프를 푼다
        setLive(true);

        // 백프레셔: GUI 가 못 따라가면 최신 것만 살리고 버린다. 큐드 연결이라
        // 밀어넣기는 항상 성공하지만, 쌓이면 화면은 과거를 보여주고 메모리만 는다.
        if (m_queued.load(std::memory_order_relaxed) < kMaxQueued) {
            m_queued.fetch_add(1, std::memory_order_relaxed);
            videofilters::apply(frame,
                m_brightness.load(std::memory_order_relaxed),
                m_contrast.load(std::memory_order_relaxed),
                m_sharpen.load(std::memory_order_relaxed),
                m_saturation.load(std::memory_order_relaxed));
            emit frameReceived(m_ch, matToQImage(frame));
        }

        msleep(1);   // CPU 과점유 방지
    }

    cap.release();
    setLive(false);
}

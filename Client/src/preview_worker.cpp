#include "preview_worker.h"

#include <QDebug>
#include <opencv2/opencv.hpp>

namespace {
// 미리보기 타일에 떠 있어도 되는 프레임 수. video_worker(3)보다 낮게 잡는다 —
// 워커가 4개라 같은 값을 주면 큐에 최대 12장이 뜬다. 미리보기는 640x480 이라
// 한 장이 ~0.9MB 로 작지만, 늦은 프레임은 감시 화면에서 가치가 없으므로 얕게 둔다.
constexpr int kMaxQueued = 2;

QImage matToQImage(const cv::Mat &mat)
{
    if (mat.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage(reinterpret_cast<const uchar *>(rgb.data), rgb.cols, rgb.rows,
                      rgb.step, QImage::Format_RGB888).copy();
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
    wait(3000);
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

    // ⚠️ qputenv 는 프로세스 전역이고 OpenCV 는 VideoCapture 를 **열 때** 읽는다.
    //    미리보기 워커가 4개 + 메인 워커까지 동시에 이 값을 쓰는데, 값은 전부
    //    같으므로(둘 다 tcp + stimeout 5초) 서로 덮어써도 결과가 달라지지 않는다.
    //    옵션 이름은 반드시 `stimeout` 이다 — FFmpeg RTSP 디먹서에서 `timeout` 은
    //    "들어오는 연결을 기다리는 시간"이라 주는 순간 클라이언트가 아니라 서버로
    //    동작하려 들고 접속조차 못 한다 (video_worker.cpp 의 같은 주석 참고).
    qputenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000");

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
        if (!cap.retrieve(frame) || frame.empty())
            continue;

        everGotFrame = true;
        retry = 0;          // 프레임이 실제로 왔을 때만 백오프를 푼다
        setLive(true);

        // 백프레셔: GUI 가 못 따라가면 최신 것만 살리고 버린다. 큐드 연결이라
        // 밀어넣기는 항상 성공하지만, 쌓이면 화면은 과거를 보여주고 메모리만 는다.
        if (m_queued.load(std::memory_order_relaxed) < kMaxQueued) {
            m_queued.fetch_add(1, std::memory_order_relaxed);
            emit frameReceived(m_ch, matToQImage(frame));
        }

        msleep(1);   // CPU 과점유 방지
    }

    cap.release();
    setLive(false);
}

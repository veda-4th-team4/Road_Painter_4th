#ifndef VIDEO_WORKER_H
#define VIDEO_WORKER_H

#include <QThread>
#include <QImage>
#include <QElapsedTimer>
#include <QPolygonF>
#include <QList>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <atomic>
#include <array>

struct StreamStats {
    bool isConnected = false;
    double fps = 0.0;
    double latencyMs = 0.0;
    int frameDropCount = 0;
};
Q_DECLARE_METATYPE(StreamStats)

// ⚠️ 여기 있던 enum FilterMode(고대비/반전/TopView/ArUco) 와 setFilterMode(),
//    setHomographyPoints() 는 전부 지웠다 — 호출하는 곳이 한 군데도 없어서
//    m_filterMode 는 항상 Original 이었다. TopView 는 VideoView 가 그리고,
//    ArUco 는 detectArucoAlways() 가 한다. 되살리지 말 것.

class video_worker : public QThread
{
    Q_OBJECT
public:
    explicit video_worker(const QString& rtspUrl, QObject *parent = nullptr);
    ~video_worker();
    void stop();

    // ⚠️ 여기 있던 setUndistort(k1,k2) / buildUndistortMaps() 는 지웠다.
    //    프레임 자체를 remap 으로 펴는 옛 2계수 경로였는데,
    //      · 캘리브 번들은 K + plumb-bob 5계수(camcalib)로 오고,
    //      · RTSP 화면은 **왜곡된 원본 그대로** 두고 그 위에 선을 왜곡시켜 그리기로
    //        확정(2026-07-29)했다.
    //    프레임을 펴 버리면 VideoView 의 정변환(topViewToOriginal→cam.distort)이
    //    이중 보정이 되어 오버레이가 어긋난다. 되살리지 말 것.

    // GUI 스레드가 프레임 하나를 소비했다고 알린다 (Backend 가 연결해 준다).
    // frameReceived 는 큐드 연결이라, GUI 가 못 따라가면 이벤트 큐에 프레임이
    // 무한히 쌓인다 — 1080p RGB 한 장이 6MB 라 지연도 메모리도 같이 늘어난다.
    // 큐에 몇 장 떠 있는지 세서 넘치면 캡처 쪽에서 버린다.
    void frameConsumed() { m_queued.fetch_sub(1, std::memory_order_relaxed); }

    void setVideoFilters(int b, int c, int s, int sat) {
        // 멀티스레드 환경이므로 안전하게 QMutex 등으로 보호해주면 더 좋습니다.
        m_mutex.lock();
        m_brightness = b;
        m_contrast = c;
        m_sharpen = s;
        m_saturation = sat;
        m_mutex.unlock();
    }

signals:
    void frameReceived(const QImage &image);
    // 상태 데이터를 UI 스레드로 보낼 시그널 추가
    void statsUpdated(const StreamStats &stats);
    // ArUco 상시 검출 결과 (원본 프레임 픽셀 좌표). 오버레이는 VideoView 가 그린다.
    void arucoDetected(const QList<int> &ids, const QList<QPolygonF> &cornersPx);
    // 스트림을 못 열었을 때. 이걸 안 올리면 화면에는 "영상 없음" 만 남아서
    // 주소가 틀린 건지 카메라가 죽은 건지 조작자가 알 방법이 없다.
    void openFailed(const QString &url);
    // 끊겼다가 스스로 다시 붙었을 때. 조작자가 "왜 잠깐 멈췄지" 를 알 수 있게 한다.
    void streamReconnected();

public:
    // ArUco 검출 on/off. **끄면 검출을 아예 안 돌린다** (표시만 숨기는 게 아니다).
    // 이 검출은 화면 표시 전용이다 — 로봇의 실제 위치는 CCTV 앱이 검출해 서버로 보내고
    // 서버가 POSE 로 내려준다. 즉 같은 일을 중복으로 하는 것이라, 필요 없으면 끄면 된다.
    // 캡처 스레드가 매 프레임 읽으므로 atomic 이다.
    void setArucoEnabled(bool on) { m_arucoEnabled.store(on, std::memory_order_relaxed); }

    // RTSP 캡처 옵션을 프로세스 전역으로 한 번 세팅한다.
    // ⚠️ **main() 에서 VideoCapture 를 만들기 전에 한 번만** 부를 것.
    //    qputenv 는 프로세스 전역이라 워커마다 부르면 서로 덮어쓴다.
    //    FFmpeg 버전에 따라 타임아웃 옵션 이름이 다른 것도 여기서 처리한다.
    // ⚠️ 이 선언을 signals: 블록 안에 두면 moc 가 시그널로 취급해
    //    "'this' is unavailable for static member functions" 로 깨진다.
    static void applyFfmpegCaptureOptions();

protected:
    void run() override;

private:
    QString m_rtspUrl;
    // 캡처 스레드가 읽고 GUI 스레드가 쓴다. 재연결 백오프 중에도 즉시 보여야 한다.
    std::atomic<bool> m_stopRequested;
    std::atomic<int>  m_queued{0};   // GUI 이벤트 큐에 떠 있는 프레임 수
    std::atomic<bool> m_arucoEnabled{true};   // 위 setArucoEnabled 참고

    // cv::Mat을 QImage로 변환하는 헬퍼 함수 (무복사 — 아래 프레임 풀과 짝이다)
    QImage matToQImage(const cv::Mat& mat);

    // 🔴 프레임 풀 — 디코더가 쓸 버퍼를 돌려쓴다.
    //
    // matToQImage 가 깊은 복사를 안 하려면(무복사) QImage 가 가리키는 버퍼가 화면에
    // 뿌려지는 동안 살아 있어야 한다. 복사를 없애면 프레임당 3.11ms 와 초당 676MB 의
    // 메모리 대역을 아낀다 (2026-08-04 실측, 당시 2592x1520 = 한 장 11.3MB).
    // 현재 프로파일 1920x1080 은 한 장 6.2MB 라 절감폭은 그 절반 남짓이지만,
    // 구조는 그대로 유효하다.
    //
    // ⚠️ **Mat 의 refcount 만 믿으면 안 된다.** cv::Mat::create 는 크기·타입이 같으면
    //    refcount 를 보지 않고 기존 버퍼를 그대로 쓴다. 즉 다음 retrieve() 가 화면에
    //    떠 있는 버퍼를 덮어쓴다 — 간헐적 화면 깨짐으로만 보여서 원인 찾기가 지옥이다.
    //    (1차 시도에서 실제로 이 함정을 밟았고, 테스트로 잡았다.)
    //    그래서 여기서 **재사용 전에 아직 공유 중인지 직접 확인**하고, 공유 중이면
    //    그 슬롯만 새로 할당한다.
    //
    // 슬롯 수는 "동시에 떠 있을 수 있는 프레임 수 + 여유". 백프레셔가 큐를 2장으로
    // 묶으므로 4면 충분하다 — 실측에서 40프레임을 돌려도 재할당이 0회였다.
    cv::Mat &nextFrameBuffer();
    // ⚠️ std::vector<cv::Mat> m_pool{4} 로 쓰지 말 것 — "원소 4개" 인지 "4 라는 원소
    //    하나" 인지 브레이스 초기화 규칙에 기대게 된다. std::array 면 그 모호함이 없다.
    static constexpr int kPoolSize = 4;
    std::array<cv::Mat, kPoolSize> m_pool;
    int m_poolIdx = 0;
    // 상시 ArUco 검출 (몇 프레임마다 1회). 사전(dictionary)은 처음 잡힌 것으로 고정.
    void detectArucoAlways(const cv::Mat &bgr);
    // 중단 요청을 100ms 마다 확인하며 잔다. 종료 요청이 들어오면 false 를 돌려준다.
    bool sleepInterruptible(int ms);

    int m_frameCounter = 0;
    int m_arucoDictLock = -1;   // 잡힌 사전 인덱스 (-1 = 탐색 중)
    int m_arucoMiss = 0;
    int m_searchFail = 0;       // 연속 탐색 실패 횟수 → 탐색 간격을 늘리는 데 쓴다
    int m_searchSkip = 0;
    int m_lastArucoCount = -1;
    // ArUco ROI 추적 — 직전에 마커를 찾은 영역(**축소본 좌표**). 다음 프레임은 여기만
    // 본다. 전체의 10% 남짓이라 검출이 27ms → 3.3ms 로 떨어진다 (2592x1520 실측).
    // 놓치면 valid 를 내리고 다음 프레임에 전체부터 다시 찾는다.
    cv::Rect m_arucoRoi;
    bool m_arucoRoiValid = false;

    QMutex m_mutex; // #include <QMutex> 필요
    int m_brightness = 0;
    int m_contrast = 0;
    int m_sharpen = 0;
    int m_saturation = 0;
};

#endif // VIDEO_WORKER_H

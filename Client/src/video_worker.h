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

protected:
    void run() override;

private:
    QString m_rtspUrl;
    // 캡처 스레드가 읽고 GUI 스레드가 쓴다. 재연결 백오프 중에도 즉시 보여야 한다.
    std::atomic<bool> m_stopRequested;
    std::atomic<int>  m_queued{0};   // GUI 이벤트 큐에 떠 있는 프레임 수

    // cv::Mat을 QImage로 변환하는 헬퍼 함수
    QImage matToQImage(const cv::Mat& mat);
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

    QMutex m_mutex; // #include <QMutex> 필요
    int m_brightness = 0;
    int m_contrast = 0;
    int m_sharpen = 0;
    int m_saturation = 0;
};

#endif // VIDEO_WORKER_H

#ifndef VIDEO_WORKER_H
#define VIDEO_WORKER_H

#include <QThread>
#include <QImage>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include <QMutex>

struct StreamStats {
    bool isConnected = false;
    double fps = 0.0;
    double latencyMs = 0.0;
    int frameDropCount = 0;
};

enum class FilterMode {
    Original,
    HighContrastGrayscale,
    HighContrastColor,
    FlipUpDown,
    TopView,
    DetectArUco
};

class video_worker : public QThread
{
    Q_OBJECT
public:
    explicit video_worker(const QString& rtspUrl, QObject *parent = nullptr);
    ~video_worker();
    void stop();
    void setFilterMode(FilterMode mode);
    void setHomographyPoints(const QList<QPoint>& points);

    // 메인 스레드에서 호출하여 값을 변경할 함수
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

protected:
    void run() override;

private:
    QString m_rtspUrl;
    FilterMode m_filterMode = FilterMode::Original;
    bool m_stopRequested;

    // cv::Mat을 QImage로 변환하는 헬퍼 함수
    QImage matToQImage(const cv::Mat& mat);

    QMutex m_mutex; // #include <QMutex> 필요
    int m_brightness = 0;
    int m_contrast = 0;
    int m_sharpen = 0;
    int m_saturation = 0;
    std::vector<cv::Point2f> m_homographySrcPoints;
    bool m_updateHomography = false;
};

#endif // VIDEO_WORKER_H

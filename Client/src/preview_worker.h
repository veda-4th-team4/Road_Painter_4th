#ifndef PREVIEW_WORKER_H
#define PREVIEW_WORKER_H

#include <QThread>
#include <QImage>
#include <QString>
#include <atomic>

// 4채널 미리보기 타일 하나를 채우는 경량 캡처 워커 (서브스트림 전용).
//
// video_worker 와 왜 따로 두는가:
//   video_worker 는 프레임을 받으면서 **같은 스레드에서 ArUco 를 검출한다.** 주석
//   (video_worker.cpp)에 따르면 1080p 에서 검출 한 번이 20~30ms 이고 사전 3개를 다
//   뒤지면 100ms 에 육박한다. 미리보기는 4개가 동시에 도는데 거기에 검출을 얹으면
//   CPU 가 그대로 녹는다 — 그리고 미리보기에서는 마커가 필요하지도 않다.
//   (마커 검출은 [작업하기]로 채널을 하나 고른 뒤 video_worker 가 담당한다)
//
// video_worker 에서 가져온 것 (같은 함정을 두 번 밟지 않기 위해):
//   · 스트림을 **처음부터** 못 열면 재시도하지 않고 포기한다.
//     Hanwha 카메라는 틀린 자격증명으로 계속 두드리면 계정을 잠근다
//     (RTSP/1.0 490 Account Blocked). 미리보기는 워커가 4개라 위험도 4배다.
//   · 백오프는 **프레임을 실제로 받았을 때만** 리셋한다. 접속은 되는데 데이터가
//     안 오는 상태에서 "열기 성공 → 즉시 리셋"을 하면 무한 재접속 루프가 된다.
//   · 프레임 큐 백프레셔. GUI 가 못 따라가면 큐드 시그널에 프레임이 무한히 쌓인다.
//
// 버린 것: ArUco 검출, 밝기/대비/샤픈 필터, FPS/지연 통계 EMA.
// 미리보기 타일에는 아무것도 안 쓰이는 값들이라 계산할 이유가 없다.
class preview_worker : public QThread
{
    Q_OBJECT
public:
    // ch 는 화면 표시·시그널 식별용이다 (URL 조립은 호출부가 이미 끝낸 상태로 준다).
    preview_worker(int ch, const QString &rtspUrl, QObject *parent = nullptr);
    ~preview_worker() override;

    int channel() const { return m_ch; }
    void stop();

    // GUI 가 프레임 하나를 소비했다고 알린다 (Backend 가 연결해 준다).
    void frameConsumed() { m_queued.fetch_sub(1, std::memory_order_relaxed); }

signals:
    void frameReceived(int ch, const QImage &image);
    // 스트림이 살아 있는지. 타일의 라벨 색으로 쓴다.
    void liveChanged(int ch, bool live);
    // 처음부터 못 열었다 = 주소나 중계가 잘못됐다. 재시도하지 않으므로 이 신호가
    // 없으면 조작자는 "왜 이 칸만 까맣지"를 알 방법이 없다.
    void openFailed(int ch, const QString &url);

protected:
    void run() override;

private:
    // 중단 요청을 100ms 마다 확인하며 잔다. 종료 요청이 들어오면 false.
    bool sleepInterruptible(int ms);

    int m_ch;
    QString m_rtspUrl;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<int> m_queued{0};   // GUI 이벤트 큐에 떠 있는 프레임 수
};

#endif // PREVIEW_WORKER_H

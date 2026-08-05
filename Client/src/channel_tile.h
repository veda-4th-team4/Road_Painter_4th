#ifndef CHANNEL_TILE_H
#define CHANNEL_TILE_H

#include <QQuickPaintedItem>
#include <QtQml/qqmlregistration.h>
#include <QImage>

// 4채널 미리보기 그리드의 한 칸.
//
// ⚠️ 여기에 VideoView 를 재사용하지 말 것. VideoView 는 작도·캘리브레이션·도면
//    변환까지 들고 있는 3000줄짜리 물건이라, 4개를 띄우면 쓰지도 않을 기능이
//    4벌 살아난다 (뷰 변환 상태, ArUco 오버레이 버퍼, 미션 폴리라인 …).
//    미리보기가 하는 일은 "받은 QImage 를 칸에 맞춰 그리고 클릭을 알리는 것"뿐이다.
class ChannelTile : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int channel READ channel WRITE setChannel NOTIFY channelChanged)
    // 이 칸이 선택돼 있는가 (클릭 → 하이라이트). 테두리는 QML 쪽에서 그린다.
    Q_PROPERTY(bool selected READ selected WRITE setSelected NOTIFY selectedChanged)
    // 영상이 실제로 흐르고 있는가. false 면 "연결 중…" 안내를 대신 그린다.
    Q_PROPERTY(bool live READ live WRITE setLive NOTIFY liveChanged)
    // 스트림을 못 열어 포기한 상태 (재시도하지 않는다 — 계정 잠김 방지)
    Q_PROPERTY(bool failed READ failed WRITE setFailed NOTIFY failedChanged)

public:
    explicit ChannelTile(QQuickItem *parent = nullptr);

    int channel() const { return m_ch; }
    void setChannel(int ch);
    bool selected() const { return m_selected; }
    void setSelected(bool v);
    bool live() const { return m_live; }
    void setLive(bool v);
    bool failed() const { return m_failed; }
    void setFailed(bool v);

    // Backend 가 preview_worker 의 프레임을 여기로 넘긴다.
    Q_INVOKABLE void onFrame(const QImage &img);

    void paint(QPainter *p) override;

signals:
    void channelChanged();
    void selectedChanged();
    void liveChanged();
    void failedChanged();
    // 사용자가 이 칸을 눌렀다. Backend.highlightChannel(ch) 로 이어진다.
    void clicked(int ch);

protected:
    void mousePressEvent(QMouseEvent *e) override;

private:
    int m_ch = 0;
    bool m_selected = false;
    bool m_live = false;
    bool m_failed = false;
    QImage m_frame;
};

#endif // CHANNEL_TILE_H

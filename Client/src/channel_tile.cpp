#include "channel_tile.h"

#include <QPainter>
#include <QMouseEvent>

ChannelTile::ChannelTile(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton);
    // 미리보기는 부드러움보다 CPU 가 중요하다 — 4칸이 동시에 갱신된다.
    setRenderTarget(QQuickPaintedItem::FramebufferObject);
}

void ChannelTile::setChannel(int ch)
{
    if (m_ch == ch) return;
    m_ch = ch;
    emit channelChanged();
    update();
}

void ChannelTile::setSelected(bool v)
{
    if (m_selected == v) return;
    m_selected = v;
    emit selectedChanged();
    update();
}

void ChannelTile::setLive(bool v)
{
    if (m_live == v) return;
    m_live = v;
    emit liveChanged();
    update();
}

void ChannelTile::setFailed(bool v)
{
    if (m_failed == v) return;
    m_failed = v;
    emit failedChanged();
    update();
}

void ChannelTile::onFrame(const QImage &img)
{
    if (img.isNull()) return;
    m_frame = img;
    update();
}

void ChannelTile::paint(QPainter *p)
{
    const QRectF box = boundingRect();
    p->fillRect(box, QColor(0x14, 0x16, 0x19));

    if (!m_frame.isNull()) {
        // 종횡비를 유지해 칸 안에 맞춘다 (레터박스). 늘리면 로봇이 실제와 다른
        // 모양으로 보여서 "저게 우리 기체가 맞나"를 눈으로 판단할 수 없게 된다.
        const QSize target = m_frame.size().scaled(box.size().toSize(), Qt::KeepAspectRatio);
        const QRectF dst(box.center().x() - target.width() / 2.0,
                         box.center().y() - target.height() / 2.0,
                         target.width(), target.height());
        p->setRenderHint(QPainter::SmoothPixmapTransform, true);
        p->drawImage(dst, m_frame);
        return;
    }

    // 아직 프레임이 없다 — 왜 없는지를 칸 안에 적는다. 그냥 까맣게 두면
    // "카메라가 죽었나 / 주소가 틀렸나 / 아직 붙는 중인가"를 구분할 수 없다.
    p->setPen(m_failed ? QColor(0xE5, 0x6B, 0x6B) : QColor(0xFF, 0xFF, 0xFF, 0x66));
    QFont f = p->font();
    f.setPixelSize(13);
    p->setFont(f);
    p->drawText(box, Qt::AlignCenter,
                m_failed
                    ? QStringLiteral("CH%1 연결 실패\n중계 주소를 확인하세요").arg(m_ch)
                    // 중계는 on-demand 라 아무도 안 보면 카메라에서 당겨오지 않는다.
                    // 첫 프레임까지 1~3초가 정상이므로 그 사이를 안내한다.
                    : QStringLiteral("CH%1 연결 중…").arg(m_ch));
}

void ChannelTile::mousePressEvent(QMouseEvent *e)
{
    emit clicked(m_ch);
    e->accept();
}

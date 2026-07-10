#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QMenu>
#include <QAction>
#include "videowidget.h"
#include "video_worker.h"
#include "videosettingsdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;
    void setupBinding();

private:
    Ui::MainWindow *ui;
    VideoWidget *m_videoWidget;
    video_worker *m_worker;
    QPushButton *btnNormal;
    QPushButton *btnMeasure;
    QPushButton *btnPathLine;
    QPushButton *btnPathAdvanced;
    QPushButton *btnDraw;
    QPushButton *btnSetting;
    QPushButton *btnFilter;

    VideoSettingsDialog::Settings m_currentVideoSettings; // 현재 설정값 보관용

private slots:
    // 통계 데이터를 받아 화면에 표시할 슬롯 추가
    void updateStreamStats(const StreamStats &stats);
    void onNormalBtnClicked();
    void onMeasureBtnClicked();
    void onPathBtnClicked();

    void onVideoConfigBtnClicked();
    void applyVideoSettings(const VideoSettingsDialog::Settings &settings);
    void handlePathDrawingFinished(const QList<QPoint>& points);
    void onCaliBtnClicked();
};
#endif // MAINWINDOW_H

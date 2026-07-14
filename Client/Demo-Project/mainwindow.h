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
#include "settingdialog.h"
#include "channelsettings.h"
#include "robotsettings.h"
#include "labdialog.h"

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

public slots:
    void toggleTestMode(bool enable); // 모드 전환 함수

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
    QPushButton *btnRobot;
    QPushButton *btnLab;
    QString m_rtspUrl; // RTSP 주소를 저장할 멤버 변수 추가

    VideoSettingsDialog::Settings m_currentVideoSettings; // 현재 설정값 보관용
    bool m_isTestMode = false; // 현재 모드 상태 저장

private slots:
    // 통계 데이터를 받아 화면에 표시할 슬롯 추가
    void updateStreamStats(const StreamStats &stats);
    void onNormalBtnClicked();
    void onMeasureBtnClicked();
    void onPathBtnClicked();

    void onLabBtnClicked();
    void changeRtspStream(const QString &newUrl); // RTSP 주소 변경 처리를 위한 슬롯 추가
    void onRobotBtnClicked();
    void onChannelBtnClicked();
    void onSettingBtnClicked();
    void onVideoConfigBtnClicked();
    void applyVideoSettings(const VideoSettingsDialog::Settings &settings);
    void handlePathDrawingFinished(const QList<QPoint>& points);
    void onCaliBtnClicked();
    void onMarkerBtnClicked();
};
#endif // MAINWINDOW_H

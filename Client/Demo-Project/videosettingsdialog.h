#ifndef VIDEOSETTINGSDIALOG_H
#define VIDEOSETTINGSDIALOG_H

#include <QDialog>
#include <QSlider>
#include <QFormLayout>
#include <QPushButton>

namespace Ui {
class VideoSettingsDialog;
}

class VideoSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit VideoSettingsDialog(QWidget *parent = nullptr);
    ~VideoSettingsDialog();
    // 현재 슬라이더 값들을 한 번에 가져오는 구조체
    struct Settings {
        int brightness = 0;  // -100 ~ 100
        int contrast = 0;    // -100 ~ 100
        int sharpen = 0;     // 0 ~ 100
        int saturation = 0;  // -100 ~ 100
    };

    void setVideoFilters(const VideoSettingsDialog::Settings &settings) {
        m_filters = settings;
        this->update(); // 값이 바뀌면 화면을 다시 그리도록 강제 새로고침
    }

    void setInitialSettings(const Settings& settings);

signals:
    // 슬라이더 조절 시 실시간으로 부모 창에 변경 값을 알림
    void settingsChanged(const VideoSettingsDialog::Settings &settings);

private:
    Ui::VideoSettingsDialog *ui;
    QSlider *m_sliderBrightness;
    QSlider *m_sliderContrast;
    QSlider *m_sliderSharpen;
    QSlider *m_sliderSaturation;
    // 필터 값을 보관할 변수 추가
    VideoSettingsDialog::Settings m_filters;
    Settings m_defaultSettings;

private slots:
    void onSliderValueChanged();
    void resetToDefault();
};

#endif // VIDEOSETTINGSDIALOG_H

#include "videosettingsdialog.h"
#include "ui_videosettingsdialog.h"

VideoSettingsDialog::VideoSettingsDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::VideoSettingsDialog)
{
    ui->setupUi(this);

    setWindowTitle("비디오 설정");

    // 슬라이더 값이 바뀔 때마다 변경 시그널 발생
    connect(ui->sliderBrightness, &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(ui->sliderContrast,   &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(ui->sliderSharpen,    &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(ui->sliderSaturation, &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(ui->CloseBtn, &QPushButton::clicked, this, &QDialog::accept);
    connect(ui->ResetBtn, &QPushButton::clicked, this, &VideoSettingsDialog::resetToDefault);
}

void VideoSettingsDialog::setInitialSettings(const Settings& settings) {
    // 초기값
    m_defaultSettings = settings;

    // UI 반영
    ui->sliderBrightness->setValue(settings.brightness);
    ui->sliderContrast->setValue(settings.contrast);
    ui->sliderSharpen->setValue(settings.sharpen);
    ui->sliderSaturation->setValue(settings.saturation);
}

// 초기화 슬롯 구현
void VideoSettingsDialog::resetToDefault() {
    ui->sliderBrightness->setValue(m_defaultSettings.brightness);
    ui->sliderContrast->setValue(m_defaultSettings.contrast);
    ui->sliderSharpen->setValue(m_defaultSettings.sharpen);
    ui->sliderSaturation->setValue(m_defaultSettings.saturation);

    // 슬라이더가 이동했으므로 변경 시그널 즉시 발생
    onSliderValueChanged();
}

void VideoSettingsDialog::onSliderValueChanged() {
    Settings currentSettings{
        ui->sliderBrightness->value(),
        ui->sliderContrast->value(),
        ui->sliderSharpen->value(),
        ui->sliderSaturation->value()
    };
    emit settingsChanged(currentSettings); // 메인 윈도우나 위젯으로 전송
}

VideoSettingsDialog::~VideoSettingsDialog()
{
    delete ui;
}

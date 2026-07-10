#include "videosettingsdialog.h"
#include "ui_videosettingsdialog.h"

VideoSettingsDialog::VideoSettingsDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::VideoSettingsDialog)
{
    ui->setupUi(this);

    setWindowTitle("비디오 설정");

    auto layout = new QFormLayout(this);

    // 슬라이더 초기화 함수 람다
    auto createSlider = [](int min, int max, int init) {
        auto slider = new QSlider(Qt::Horizontal);
        slider->setRange(min, max);
        slider->setValue(init);
        return slider;
    };

    m_sliderBrightness = createSlider(-100, 100, 0);
    m_sliderContrast   = createSlider(-100, 100, 0);
    m_sliderSharpen    = createSlider(0, 100, 0);
    m_sliderSaturation = createSlider(-100, 100, 0);

    layout->addRow("밝기 (Brightness):", m_sliderBrightness);
    layout->addRow("대비 (Contrast):", m_sliderContrast);
    layout->addRow("선명도 (Sharpen):", m_sliderSharpen);
    layout->addRow("채도 (Saturation):", m_sliderSaturation);

    auto btnClose = new QPushButton("닫기", this);
    layout->addRow(btnClose);

    // 슬라이더 값이 바뀔 때마다 변경 시그널 발생
    connect(m_sliderBrightness, &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(m_sliderContrast,   &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(m_sliderSharpen,    &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(m_sliderSaturation, &QSlider::valueChanged, this, &VideoSettingsDialog::onSliderValueChanged);
    connect(btnClose, &QPushButton::clicked, this, &QDialog::accept);
}

void VideoSettingsDialog::setInitialSettings(const Settings& settings) {
    m_sliderBrightness->setValue(settings.brightness);
    m_sliderContrast->setValue(settings.contrast);
    m_sliderSharpen->setValue(settings.sharpen);
    m_sliderSaturation->setValue(settings.saturation);
}

void VideoSettingsDialog::onSliderValueChanged() {
    Settings currentSettings{
        m_sliderBrightness->value(),
        m_sliderContrast->value(),
        m_sliderSharpen->value(),
        m_sliderSaturation->value()
    };
    emit settingsChanged(currentSettings); // 메인 윈도우나 위젯으로 전송
}

VideoSettingsDialog::~VideoSettingsDialog()
{
    delete ui;
}

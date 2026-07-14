#include "settingdialog.h"
#include "ui_settingdialog.h"

SettingDialog::SettingDialog(bool isTestMode, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SettingDialog)
    , m_isTestMode(isTestMode)
{
    ui->setupUi(this);
    setWindowTitle("설정");

    // 1. 이미 디자인된 ui->BtnChMod 버튼의 텍스트를 현재 모드에 맞춰 설정
    if (m_isTestMode) {
        ui->BtnChMod->setText("일반 모드로 돌아가기");
    } else {
        ui->BtnChMod->setText("테스트 모드 진입하기");
    }

    // 2. 이미 디자인된 ui->BtnChMod 버튼의 클릭 이벤트 연결
    connect(ui->BtnChMod, &QPushButton::clicked, this, [this]() {
        emit requestModeChange(!m_isTestMode); // 모드 전환 요청 발생
        accept(); // 다이얼로그 닫기
    });
}

SettingDialog::~SettingDialog()
{
    delete ui;
}

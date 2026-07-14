#include "labdialog.h"
#include "ui_labdialog.h"
#include <QMessageBox>

LabDialog::LabDialog(const QString &currentUrl, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::LabDialog)
{
    ui->setupUi(this);
    setWindowTitle("Lab..");

    // 1. UI 상의 RTSPlineedit에 현재 연결되어 있던 주소를 먼저 채워줍니다.
    ui->RTSPlineedit->setText(currentUrl);

    // 2. RTSPCgnBtn 버튼이 클릭되었을 때의 이벤트 연결
    connect(ui->RTSPCgnBtn, &QPushButton::clicked, this, &LabDialog::onRTSPCgnBtnClicked);


    connect(ui->CloseBtn, &QPushButton::clicked, this, &QDialog::accept);
}

LabDialog::~LabDialog()
{
    delete ui;
}

void LabDialog::onRTSPCgnBtnClicked()
{
    // 공백을 제거한 입력 값을 가져옵니다.
    QString newUrl = ui->RTSPlineedit->text().trimmed();

    if (newUrl.isEmpty()) {
        QMessageBox::warning(this, "입력 오류", "RTSP 주소가 비어있습니다. 주소를 입력해 주세요.");
        return;
    }

    // 메인 윈도우로 신호를 보내고 창을 닫습니다.
    emit rtspUrlChanged(newUrl);
    accept();
}
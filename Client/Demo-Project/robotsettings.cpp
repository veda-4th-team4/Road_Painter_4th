#include "robotsettings.h"
#include "ui_robotsettings.h"

RobotSettings::RobotSettings(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RobotSettings)
{
    ui->setupUi(this);
    setWindowTitle("로봇 환경설정");

    connect(ui->CloseBtn, &QPushButton::clicked, this, &QDialog::accept);
}

RobotSettings::~RobotSettings()
{
    delete ui;
}

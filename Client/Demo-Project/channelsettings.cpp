#include "channelsettings.h"
#include "ui_channelsettings.h"

ChannelSettings::ChannelSettings(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ChannelSettings)
{
    ui->setupUi(this);
    setWindowTitle("채널 설정");

    connect(ui->CloseBtn, &QPushButton::clicked, this, &QDialog::accept);
}

ChannelSettings::~ChannelSettings()
{
    delete ui;
}

#ifndef CHANNELSETTINGS_H
#define CHANNELSETTINGS_H

#include <QDialog>

namespace Ui {
class ChannelSettings;
}

class ChannelSettings : public QDialog
{
    Q_OBJECT

public:
    explicit ChannelSettings(QWidget *parent = nullptr);
    ~ChannelSettings();

private:
    Ui::ChannelSettings *ui;
};

#endif // CHANNELSETTINGS_H

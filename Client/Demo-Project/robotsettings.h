#ifndef ROBOTSETTINGS_H
#define ROBOTSETTINGS_H

#include <QDialog>

namespace Ui {
class RobotSettings;
}

class RobotSettings : public QDialog
{
    Q_OBJECT

public:
    explicit RobotSettings(QWidget *parent = nullptr);
    ~RobotSettings();

private:
    Ui::RobotSettings *ui;
};

#endif // ROBOTSETTINGS_H

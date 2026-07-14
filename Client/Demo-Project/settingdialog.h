#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H

#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>

namespace Ui {
class SettingDialog;
}

class SettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingDialog(bool isTestMode,QWidget *parent = nullptr);
    ~SettingDialog();

signals:
    void requestModeChange(bool enable); // 모드 변경 요청 시그널

private:
    Ui::SettingDialog *ui;
    bool m_isTestMode;
};

#endif // SETTINGDIALOG_H

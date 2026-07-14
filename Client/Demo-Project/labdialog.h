#ifndef LABDIALOG_H
#define LABDIALOG_H

#include <QDialog>

namespace Ui {
class LabDialog;
}

class LabDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LabDialog(const QString &currentUrl,QWidget *parent = nullptr);
    ~LabDialog();

signals:
    // 메인 윈도우로 새 주소를 던져줄 시그널 선언
    void rtspUrlChanged(const QString &newUrl);

private:
    Ui::LabDialog *ui;

private slots:
    // RTSPCgnBtn 클릭 핸들러
    void onRTSPCgnBtnClicked();
};

#endif // LABDIALOG_H

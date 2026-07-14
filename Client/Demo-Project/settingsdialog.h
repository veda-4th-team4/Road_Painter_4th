#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>

namespace Ui {
class SettingsDialog;
}

class SettingsDialog : public QDialog // QDialog 상속 확인
{
    Q_OBJECT
public:
    // 헤더 파일에서 직접 선언 및 구현 (inline)
    explicit SettingsDialog(bool isTestMode, QWidget *parent = nullptr) : QDialog(parent) {
        // QVBoxLayout *layout = new QVBoxLayout(this);
        QPushButton *btnModeToggle = new QPushButton(isTestMode ? "일반 모드로 돌아가기" : "테스트 모드 진입하기", this);
        // layout->addWidget(btnModeToggle);

        connect(btnModeToggle, &QPushButton::clicked, this, [this, isTestMode]() {
            emit requestModeChange(!isTestMode);
            accept();
        });
    }

signals:
    void requestModeChange(bool enable);
};


#endif // SETTINGSDIALOG_H
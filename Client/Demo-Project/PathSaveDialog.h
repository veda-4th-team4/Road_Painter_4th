#ifndef PATHSAVEDIALOG_H
#define PATHSAVEDIALOG_H

#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

class PathSaveDialog : public QDialog {
    Q_OBJECT

public:
    explicit PathSaveDialog(QWidget *parent = nullptr) : QDialog(parent) {
        setWindowTitle("경로 작도");
        setFixedSize(350, 150);

        QLabel *lblMessage = new QLabel("경로 작도가 완료되었습니다.", this);
        lblMessage->setAlignment(Qt::AlignCenter);

        btnSaveTemp = new QPushButton("저장하기", this);
        btnExportFile = new QPushButton("파일로\n내보내기", this);
        // btnCalibrate = new QPushButton("Top View 보정\n(호모그래피 적용)", this);
        btnCancel = new QPushButton("취소", this);

        // 버튼 높이 조절로 다이얼로그 가독성 향상
        btnSaveTemp->setFixedHeight(45);
        btnExportFile->setFixedHeight(45);
        //  btnCalibrate->setFixedHeight(45);
        btnCancel->setFixedHeight(45);

        QHBoxLayout *btnLayout = new QHBoxLayout();
        btnLayout->addWidget(btnSaveTemp);
        btnLayout->addWidget(btnExportFile);
        // btnLayout->addWidget(btnCalibrate);
        btnLayout->addWidget(btnCancel);

        QVBoxLayout *mainLayout = new QVBoxLayout(this);
        mainLayout->addWidget(lblMessage);
        mainLayout->addLayout(btnLayout);

        // 시그널 연결 (QDialog의 내장 결과값 활용)
        connect(btnSaveTemp, &QPushButton::clicked, this, [this]() { done(1); });      // 1: 임시 저장
        connect(btnExportFile, &QPushButton::clicked, this, [this]() { done(2); });    // 2: 내보내기
        //  connect(btnCalibrate, &QPushButton::clicked, this, [this]() { done(3); });     //v 3: 호모그래피 보정 저장
        connect(btnCancel, &QPushButton::clicked, this, [this]() { reject(); });
    }

private:
    QPushButton *btnSaveTemp;
    QPushButton *btnExportFile;
    QPushButton *btnCalibrate;
    QPushButton *btnCancel;
};

#endif // PATHSAVEDIALOG_H
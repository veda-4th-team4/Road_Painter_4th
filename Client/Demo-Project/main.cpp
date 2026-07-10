#include "mainwindow.h"

#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // QSS 파일 로드 및 전체 애플리케이션에 적용
    QFile file(":/custom.qss"); // 리소스에 등록된 경로
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream stream(&file);
        a.setStyleSheet(stream.readAll());
        file.close();
    } else {
        qDebug() << "Hello.qss 파일을 불러올 수 없습니다. 경로를 확인하세요.";
    }

    MainWindow w;
    w.show();
    return QCoreApplication::exec();
}

#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "pathsavedialog.h"
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QMessageBox>
#include <QDir>
#include <opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), m_worker(nullptr)
{
    ui->setupUi(this);

    this->setWindowTitle("Road-Painter (Demo Version) v2");
    m_videoWidget = ui->label;
    setupBinding();

    m_rtspUrl = "rtsp://stream.strba.sk:1935/strba/VYHLAD_JAZERO.stream";
    m_worker = new video_worker(m_rtspUrl, this);

    connect(m_worker, &video_worker::statsUpdated, this, &MainWindow::updateStreamStats);
    connect(m_worker, &video_worker::frameReceived, m_videoWidget, &VideoWidget::updateBackground);

    // 5. 스트리밍 시작
    m_worker->start();

    ui->btnChannel->setVisible(false);
    ui->MarkerBtn->setVisible(false);
    ui->btnLab->setVisible(false);
    ui->btnProfile->setVisible(false);
}

MainWindow::~MainWindow()
{
    if (m_worker) {
        m_worker->stop();
        m_worker->wait(); // 스레드가 안전하게 종료될 때까지 대기
    }
    delete ui;
}

void MainWindow::toggleTestMode(bool enable) {
    m_isTestMode = enable;

    // 테스트 모드 전용 버튼들 (예: btnDebug1, btnDebug2)
    ui->btnChannel->setVisible(enable);
    ui->MarkerBtn->setVisible(enable);
    ui->btnLab->setVisible(enable);
    ui->btnProfile->setVisible(enable);

    // 상태 표시
    if (enable) {
        this->statusBar()->showMessage("테스트 모드 활성화됨");
    } else {
        this->statusBar()->showMessage("일반 모드로 복귀");
    }
}

// 하단 상태바에 네트워크 상태 정보 출력
void MainWindow::updateStreamStats(const StreamStats &stats) {
    QString statusText = stats.isConnected ? "연결됨"
                                           : "연결 끊김";

    QString info = QString("상태: %1 | FPS: %2 | 지연: %3 ms | 프레임 드롭: %4")
                       .arg(statusText)
                       .arg(stats.fps, 0, 'f', 1)
                       .arg(stats.latencyMs, 0, 'f', 1)
                       .arg(stats.frameDropCount);

    this->statusBar()->showMessage(info);
}

void MainWindow::setupBinding() {
    // 버튼 시그널 슬롯 바인딩
    connect(ui->btnNormal, &QPushButton::clicked, this, &MainWindow::onNormalBtnClicked);
    connect(ui->btnMeasure, &QPushButton::clicked, this, &MainWindow::onMeasureBtnClicked);
    connect(ui->btnPathLine, &QPushButton::clicked, this, &MainWindow::onPathBtnClicked);
    connect(ui->btnSettings, &QPushButton::clicked, this, &MainWindow::onSettingBtnClicked);
    connect(ui->btnChannel, &QPushButton::clicked, this, &MainWindow::onChannelBtnClicked);
    connect(ui->btnRobot, &QPushButton::clicked, this, &MainWindow::onRobotBtnClicked);
    connect(ui->btnLab, &QPushButton::clicked, this, &MainWindow::onLabBtnClicked);

    // 비디오 설정 버튼 바인딩 추가
    connect(ui->btnVideoConfig, &QPushButton::clicked, this, &MainWindow::onVideoConfigBtnClicked);
    connect(m_videoWidget, &VideoWidget::pathDrawingFinished, this, &MainWindow::handlePathDrawingFinished);
    connect(ui->CaliBtn, &QPushButton::clicked, this, &MainWindow::onCaliBtnClicked);
    connect(ui->MarkerBtn, &QPushButton::clicked, this, &MainWindow::onMarkerBtnClicked);
}

void MainWindow::onLabBtnClicked() {
    // 💡 비모달 다이얼로그 생성 시 현재 RTSP 주소(m_rtspUrl)를 넘겨줍니다.
    LabDialog *dialog = new LabDialog(m_rtspUrl, this);
    dialog->setAttribute(Qt::WA_DeleteOnClose); // 창이 닫히면 메모리 자동 해제

    // 💡 다이얼로그에서 주소 변경 시그널이 오면 메인 윈도우의 스레드 재시작 슬롯과 연결
    connect(dialog, &LabDialog::rtspUrlChanged, this, &MainWindow::changeRtspStream);

    dialog->show(); // 화면에 띄우기
}

// 3. 💡 [신규 구현] 실제 RTSP 스트림을 안전하게 교체하는 로직
void MainWindow::changeRtspStream(const QString &newUrl) {
    if (newUrl.isEmpty() || m_rtspUrl == newUrl) return; // 주소가 비었거나 같으면 무시

    m_rtspUrl = newUrl; // 새 주소 갱신

    // 1단계: 기존에 동작 중이던 스레드를 안전하게 멈추고 메모리 해제
    if (m_worker) {
        m_worker->stop();
        m_worker->wait();
        delete m_worker;
        m_worker = nullptr;
    }

    // 2단계: 변경된 주소로 새로운 스레드 객체 생성
    m_worker = new video_worker(m_rtspUrl, this);

    // 3단계: 시그널-슬롯 재연결
    connect(m_worker, &video_worker::statsUpdated, this, &MainWindow::updateStreamStats);
    connect(m_worker, &video_worker::frameReceived, m_videoWidget, &VideoWidget::updateBackground);

    // 4단계: ⭐️ 중요! 새 스트림이 열려도 기존에 조절해 둔 영상 필터값(밝기, 대비 등) 유지 적용
    m_worker->setVideoFilters(m_currentVideoSettings.brightness,
                              m_currentVideoSettings.contrast,
                              m_currentVideoSettings.sharpen,
                              m_currentVideoSettings.saturation);

    // 5단계: 재생 시작
    m_worker->start();

    this->statusBar()->showMessage("RTSP 스트림 주소 변경 완료: " + m_rtspUrl);
}

void MainWindow::onRobotBtnClicked() {
    RobotSettings *dialog = new RobotSettings(this);
    dialog->show(); // 화면에 띄우기
}

void MainWindow::onSettingBtnClicked() {
    // 현재 모드 상태(m_isTestMode)를 넘겨주며 다이얼로그 생성
    SettingDialog dlg(m_isTestMode, this);

    // 다이얼로그에서 모드 변경 요청 신호가 들어오면 메인 윈도우의 toggleTestMode와 실시간 연결
    connect(&dlg, &SettingDialog::requestModeChange, this, &MainWindow::toggleTestMode);

    dlg.exec(); // 다이얼로그 모달 창 띄우기
}

void MainWindow::onChannelBtnClicked() {
    ChannelSettings *dialog = new ChannelSettings(this);
    dialog->show(); // 화면에 띄우기
}

void MainWindow::onVideoConfigBtnClicked() {
    // 다이얼로그 생성 (비모달 또는 모달 선택 가능, 여기서는 화면을 보며 조절하도록 비모달로 추천)
    VideoSettingsDialog *dialog = new VideoSettingsDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose); // 닫히면 메모리 자동 해제

    // 기존 설정값 세팅
    dialog->setInitialSettings(m_currentVideoSettings);

    // 슬라이더가 바뀔 때마다 영상 처리 슬롯으로 연결
    connect(dialog, &VideoSettingsDialog::settingsChanged, this, &MainWindow::applyVideoSettings);

    dialog->show(); // 화면에 띄우기
}

void MainWindow::applyVideoSettings(const VideoSettingsDialog::Settings &settings) {
    m_currentVideoSettings = settings;

    if (m_worker) {
        m_worker->setVideoFilters(settings.brightness,
                                  settings.contrast,
                                  settings.sharpen,
                                  settings.saturation);
    }
}

void MainWindow::onCaliBtnClicked() {
    if (!m_worker) return;

    // static 변수를 사용하여 버튼을 누를 때마다 켜짐/꺼짐 상태를 전환합니다.
    static bool isTopViewActive = false;
    isTopViewActive = !isTopViewActive;

    if (isTopViewActive) {
        // 1. 프로젝트 폴더의 임시 경로 지정 파일(JSON) 읽기 시도
        QString loadFilePath = QDir::current().absoluteFilePath("path_temp.json");
        QFile file(loadFilePath);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "오류", "path_temp.json 파일을 찾을 수 없습니다.\n화면에서 4개의 모서리를 경로 지정한 후 먼저 '저장하기'를 완료해주세요.");
            isTopViewActive = false; // 상태 원복
            return;
        }

        // 2. JSON 파싱
        QByteArray jsonData = file.readAll();
        file.close();

        QJsonDocument document = QJsonDocument::fromJson(jsonData);
        QJsonObject rootObj = document.object();
        QJsonArray pathArray = rootObj["path"].toArray();

        // 💡 호모그래피 보정은 완벽한 사각형을 위한 4개의 꼭짓점이 필수!
        if (pathArray.size() != 4) {
            QMessageBox::warning(this, "데이터 부족",
                                 QString("Top View 보정은 정확히 4개의 모서리 포인트가 필요합니다.\n현재 저장된 포인트 개수: %1개").arg(pathArray.size()));
            isTopViewActive = false;
            return;
        }

        // 3. JSON에서 x, y 좌표 추출
        QList<QPoint> loadedPoints;
        for (int i = 0; i < pathArray.size(); ++i) {
            QJsonObject ptObj = pathArray[i].toObject();
            loadedPoints.append(QPoint(ptObj["x"].toInt(), ptObj["y"].toInt()));
        }

        // 4. Worker 스레드에 좌표 전달 및 모드 활성화
        m_worker->setHomographyPoints(loadedPoints);
        m_worker->setFilterMode(FilterMode::TopView);

        this->statusBar()->showMessage("실시간 Top View 보정 모드 가동 중 (JSON 데이터 적용 완료)");
        ui->CaliBtn->setText("원본 View 보기");

    } else {
        // 원본 영상으로 복귀
        m_worker->setFilterMode(FilterMode::Original);
        this->statusBar()->showMessage("원본 스트림 모드로 복귀");
        ui->CaliBtn->setText("Top View 보기");
    }
}

// void MainWindow::on_btnSettings_clicked() {
//     SettingsDialog dlg(m_isTestMode, this); // 모드 상태 전달
//     connect(&dlg, &SettingsDialog::requestModeChange, this, &MainWindow::toggleTestMode);
//     dlg.exec();
// }

// 기본 대기 마우스 핸들러
void MainWindow::onNormalBtnClicked() {
    m_videoWidget->setInteractionMode(InteractionMode::Normal);
    this->statusBar()->showMessage("기본 상태 전환: 기능 취소 및 화면 리셋");
}

// 실시간 좌표 감지 활성화 핸들러
void MainWindow::onMeasureBtnClicked() {
    m_videoWidget->setInteractionMode(InteractionMode::MeasureCoordinate);
    this->statusBar()->showMessage("좌표 측정 모드가 켜졌습니다.");
}

// 두 좌표 선분 생성 경로지정 제어기
void MainWindow::onPathBtnClicked() {
    m_videoWidget->setInteractionMode(InteractionMode::DrawLinePath);
    this->statusBar()->showMessage("경로 지정 (직선) 모드: 화면을 클릭해 선 경로를 생성하세요.");
}

void MainWindow::handlePathDrawingFinished(const QList<QPoint>& points) {
    if (points.isEmpty()) return;

    // 1. 커스텀 다이얼로그 창 띄우기
    PathSaveDialog dialog(this);
    int result = dialog.exec(); // 사용자가 버튼을 누를 때까지 블록됨

    if (result == 0) {
        // 취소를 눌렀을 때의 처리
        this->statusBar()->showMessage("경로 저장 취소됨");
        return;
    }

    QString saveFilePath = "";

    bool applyHomography = false;

    if (result == 1) {
        saveFilePath = QDir::current().absoluteFilePath("path_temp.json");
    }
    else if (result == 2) {
        saveFilePath = QFileDialog::getSaveFileName(this, "경로 작도", QDir::currentPath(), "JSON 파일 (*.json)");
        if (saveFilePath.isEmpty()) return;
    }
    else if (result == 3) { // 💡 호모그래피 보정 기능 선택 시
        saveFilePath = QDir::current().absoluteFilePath("path_topview_temp.json");
        applyHomography = true;
    }
    // JSON 배열을 담을 객체
    QJsonArray jsonPathArray;

    // 💡 호모그래피(Top View) 변환 로직
    // =======================================================
    if (applyHomography) {
        // 1. CCTV 원본 영상 기준 4개의 좌표 설정 (예: 주차장 구역의 모서리 4개)
        // [주의] 이 좌표는 사용하시는 실제 CCTV 화면에 맞게 수정해야 정확한 보정이 됩니다.
        std::vector<cv::Point2f> srcPoints = {
            cv::Point2f(300, 200), // 좌상단
            cv::Point2f(900, 200), // 우상단
            cv::Point2f(1100, 600), // 우하단
            cv::Point2f(100, 600)  // 좌하단
        };

        // 2. 변환될 Top View 평면 기준 4개의 좌표 (예: 가로 400, 세로 600 크기의 직사각형 형태)
        std::vector<cv::Point2f> dstPoints = {
            cv::Point2f(0, 0),     // 좌상단
            cv::Point2f(400, 0),   // 우상단
            cv::Point2f(400, 600), // 우하단
            cv::Point2f(0, 600)    // 좌하단
        };

        // 3. 변환 행렬 H 계산
        cv::Mat H = cv::findHomography(srcPoints, dstPoints);

        if (!H.empty()) {
            // 사용자가 찍은 포인트들을 OpenCV 포맷으로 복사
            std::vector<cv::Point2f> originalPath;
            for (const auto& pt : points) {
                originalPath.push_back(cv::Point2f(pt.x(), pt.y()));
            }

            // 호모그래피 연산 일괄 적용 (좌표계 변환)
            std::vector<cv::Point2f> topViewPath;
            cv::perspectiveTransform(originalPath, topViewPath, H);

            // 변환된 좌표를 JSON 객체로 삽입
            for (int i = 0; i < topViewPath.size(); ++i) {
                QJsonObject pointObject;
                pointObject["sequence"] = i + 1;
                pointObject["x"] = std::round(topViewPath[i].x); // 소수점 반올림 처리
                pointObject["y"] = std::round(topViewPath[i].y);
                pointObject["is_top_view"] = true; // 보정된 데이터임을 표시
                jsonPathArray.append(pointObject);
            }
        } else {
            QMessageBox::warning(this, "보정 실패", "호모그래피 행렬을 계산할 수 없습니다.");
            return;
        }
    }
    else { // 기본(원본) 좌표계 저장 로직
        for (int i = 0; i < points.size(); ++i) {
            QJsonObject pointObject;
            pointObject["sequence"] = i + 1;
            pointObject["x"] = points[i].x();
            pointObject["y"] = points[i].y();
            pointObject["is_top_view"] = false;
            jsonPathArray.append(pointObject);
        }
    }

    QJsonObject rootObject;
    rootObject["total_points"] = points.size();
    rootObject["path"] = jsonPathArray;

    // JSON 문서 객체 생성 및 인덴트가 적용된 텍스트 포맷 변환
    QJsonDocument jsonDoc(rootObject);

    // 4. 파일 저장 수행
    QFile file(saveFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "오류", "파일을 생성하거나 열 수 없습니다:\n" + saveFilePath);
        return;
    }

    file.write(jsonDoc.toJson(QJsonDocument::Indented)); // 보기 좋게 들여쓰기하여 저장
    file.close();

    this->statusBar()->showMessage("작도 경로 저장 완료: " + saveFilePath);
    QMessageBox::information(this, "저장 완료", "성공적으로 경로 정보가 저장되었습니다.\n파일 경로: " + saveFilePath);
}

void MainWindow::onMarkerBtnClicked() {
    if (!m_worker) return;

    // static 변수를 사용하여 토글 스위치처럼 작동하도록 만듭니다.
    static bool isMarkerActive = false;
    isMarkerActive = !isMarkerActive;

    if (isMarkerActive) {
        m_worker->setFilterMode(FilterMode::DetectArUco);
        this->statusBar()->showMessage("실시간 ArUco 마커 검출 모드 가동 중");
        ui->MarkerBtn->setText("마커 검출 중지"); // 켜졌을 때 버튼 텍스트 변경
    } else {
        m_worker->setFilterMode(FilterMode::Original);
        this->statusBar()->showMessage("원본 스트림 모드로 복귀");
        ui->MarkerBtn->setText("마커 검출 (ArUco)");
    }
}
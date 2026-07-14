#include "video_worker.h"
#include <QDebug>
#include <opencv2/objdetect/aruco_detector.hpp>

video_worker::video_worker(const QString& rtspUrl, QObject *parent)
    : QThread(parent), m_rtspUrl(rtspUrl), m_stopRequested(false) {
    // Custom 구조체를 Signal/Slot에서 사용하기 위해 등록
    qRegisterMetaType<StreamStats>();
}

video_worker::~video_worker() {
    stop();
    wait();
}

void video_worker::stop() {
    m_stopRequested = true;
}

void video_worker::run() {
    StreamStats stats;
    std::string url = m_rtspUrl.toStdString();

    cv::VideoCapture cap(url, cv::CAP_FFMPEG);
    cv::Mat frame;

    if (!cap.isOpened()) {
        stats.isConnected = false;
        emit statsUpdated(stats);
        qDebug() << "RTSP 스트림을 열 수 없습니다:" << m_rtspUrl;
        return;
    }

    stats.isConnected = true;
    emit statsUpdated(stats);

    // FPS 및 Latency 측정을 위한 타이머 변수들
    QElapsedTimer fpsTimer;
    fpsTimer.start();
    int frameCount = 0;

    while (!m_stopRequested) {
        int64 tStart = cv::getTickCount();

        // 1. 프레임 읽기 시도 (grab / retrieve 분리형으로 통일)
        if (!cap.grab()) {
            stats.frameDropCount++;
            stats.isConnected = false;
            emit statsUpdated(stats);
            msleep(30);
            continue;
        }

        stats.isConnected = true; // grab 성공 시 연결 유지로 판단

        if (!cap.retrieve(frame) || frame.empty()) {
            stats.frameDropCount++;
            emit statsUpdated(stats);
            continue;
        }

        // 2. Latency 측정 (프레임을 디코딩하고 가져오기까지 걸린 시간 계산)
        int64 tEnd = cv::getTickCount();
        stats.latencyMs = ((tEnd - tStart) * 1000.0) / cv::getTickFrequency();

        // [핵심 추가] 만약 디코딩된 Latency나 처리 지연이 RTSP 스트림 속도(예: 33ms)보다 너무 길어지면
        // 버퍼가 밀리고 있다는 뜻이므로, 다음 루프에서 누적된 버퍼를 강제로 한 번 더 grab해서 날려버립니다.
        // 이 로직이 있어야 실제로 밀리는 프레임이 Drop되면서 카운터가 올라갑니다.
        if (stats.latencyMs > 60.0) { // 기준 지연 시간(ms) - 환경에 맞게 조절 가능
            cap.grab(); // 버퍼에 쌓인 다음 낡은 프레임을 강제로 건너뜀 (Drop)
            stats.frameDropCount++;
        }

        // 3. FPS 계산 (1초마다 갱신)
        frameCount++;
        if (fpsTimer.elapsed() >= 1000) {
            stats.fps = (frameCount * 1000.0) / fpsTimer.elapsed();
            frameCount = 0;
            fpsTimer.restart();

            // 1초 주기로 UI에 통계 데이터 전송 (잦은 UI 갱신으로 인한 부하 방지)
            emit statsUpdated(stats);
        }


        // --- [영상 변형 시작] ---
        m_mutex.lock();
        int b = m_brightness;
        int c = m_contrast;
        int s = m_sharpen;
        int sat = m_saturation;
        m_mutex.unlock();

        // 1. 밝기 및 대비 조절
        double alpha = (c / 100.0) + 1.0;
        frame.convertTo(frame, -1, alpha, b);

        // 2. 채도 조절
        if (sat != 0) {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            for (int r = 0; r < hsv.rows; ++r) {
                for (int col = 0; col < hsv.cols; ++col) {
                    auto &pixel = hsv.at<cv::Vec3b>(r, col);
                    pixel[1] = cv::saturate_cast<uchar>(pixel[1] + sat);
                }
            }
            cv::cvtColor(hsv, frame, cv::COLOR_HSV2BGR);
        }

        // 3. 선명도 조절
        if (s > 0) {
            cv::Mat blurred;
            cv::GaussianBlur(frame, blurred, cv::Size(9, 9), s / 10.0);
            cv::addWeighted(frame, 1.5, blurred, -0.5, 0, frame);
        }
        // --- [영상 변형 끝] ---

        // --- 원본 프레임 복사본 생성 후 영상 필터 조건 처리 ---
        cv::Mat processedFrame = frame.clone();

        if (m_filterMode == FilterMode::HighContrastGrayscale) {
            cv::Mat gray, thresh;
            // 흑백 변환
            cv::cvtColor(processedFrame, gray, cv::COLOR_BGR2GRAY);
            // 적응형 이진화 알고리즘(Adaptive Thresholding) 적용 (블록 크기는 홀수 11~15 권장)
            cv::adaptiveThreshold(gray, thresh, 255,
                                  cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv::THRESH_BINARY, 11, 2);
            // 안전하게 포맷 호환을 위해 1채널 흑백을 다시 3채널 BGR 형식으로 복사
            cv::cvtColor(thresh, processedFrame, cv::COLOR_GRAY2BGR);
        }
        else if (m_filterMode == FilterMode::HighContrastColor) {
            // 컬러 고대비 처리 (YCrCb 분할 후 Y채널 히스토그램 평활화)
            cv::Mat ycrcb;
            cv::cvtColor(processedFrame, ycrcb, cv::COLOR_BGR2YCrCb);
            std::vector<cv::Mat> channels;
            cv::split(ycrcb, channels);
            cv::equalizeHist(channels[0], channels[0]); // 밝기 채널 고대비화
            cv::merge(channels, ycrcb);
            cv::cvtColor(ycrcb, processedFrame, cv::COLOR_YCrCb2BGR);
        }
        else if (m_filterMode == FilterMode::FlipUpDown) {
            // 0은 상하반전(X축 기준 flip), 1은 좌우반전(Y축 기준 flip)입니다.
            cv::flip(processedFrame, processedFrame, -1);
        }// 💡 실시간 호모그래피 (Top View) 필터 추가
        else if (m_filterMode == FilterMode::TopView) {
            static cv::Mat H;

            // 좌표가 새로 들어왔거나(처음 켰을 때) H가 비어있을 때만 연산 (부하 방지)
            if (m_updateHomography || H.empty()) {
                if (m_homographySrcPoints.size() == 4) {
                    // 출력될 Top View 평면의 크기 (400x600 고정 예시)
                    std::vector<cv::Point2f> dstPoints = {
                        cv::Point2f(0, 0), cv::Point2f(500, 0),
                        cv::Point2f(500, 500), cv::Point2f(0, 500)
                    };

                    // JSON 파일에서 가져온 사용자의 실제 클릭 좌표로 행렬 연산
                    H = cv::findHomography(m_homographySrcPoints, dstPoints);
                }
                m_updateHomography = false;
            }


            if (!H.empty()) {
                cv::Mat warped;
                cv::warpPerspective(processedFrame, warped, H, cv::Size(500, 500));
                processedFrame = warped;
            } else {
                // 4개의 점이 아닐 경우 등 에러 상황 시 원본 영상 위에 경고 문구 출력
                cv::putText(processedFrame, "Homography Error: Need exactly 4 points in JSON",
                            cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
        }
        else if (m_filterMode == FilterMode::DetectArUco) {
            // 1. 사용할 ArUco 사전(Dictionary) 정의 (가장 대중적인 6x6 사이즈, 250개 ID)
            // ※ OpenCV 4.7+ 부터 ArUco 가 objdetect 모듈로 이동. ArucoDetector 클래스 방식 사용.
            //    Detector 는 매 프레임 새로 만들 필요가 없으므로 static 으로 1회 생성.
            static const cv::aruco::Dictionary dictionary =
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            static const cv::aruco::ArucoDetector detector(dictionary,
                                                           cv::aruco::DetectorParameters());

            // 검출된 마커의 모서리 좌표들과 ID를 담을 벡터
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<int> markerIds;

            // 2. 마커 검출 수행
            detector.detectMarkers(processedFrame, markerCorners, markerIds);

            // 3. 마커가 화면에 1개 이상 검출되었다면 초록색 테두리와 ID 번호 그리기
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(processedFrame, markerCorners, markerIds);

                // (선택) 좌측 상단에 검출된 마커 개수 텍스트 표시
                std::string msg = "Detected Markers: " + std::to_string(markerIds.size());
                cv::putText(processedFrame, msg, cv::Point(20, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
            }
        }

        // 상단 데이터 및 자막 표시 텍스트 그리기 (필터 처리된 영상 위에 오버레이)
        // std::string text = "FPS: " + std::to_string(stats.fps).substr(0, 4) +"  Latency: " + std::to_string(stats.latencyMs).substr(0, 4) + "ms";
        // cv::putText(processedFrame, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        if (!stats.isConnected) {
            cv::putText(processedFrame, "DISCONNECTED", cv::Point(20, 80),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2); // 적색
        }

        // 4. 최종 결과 영상 변환 및 UI 전송 (반드시 연산이 다 끝난 processedFrame을 전송!)
        QImage qimg = matToQImage(processedFrame);
        emit frameReceived(qimg);

        // RTSP의 자체 스트림 속도에 맞추되, CPU 과점유 방지용 미세 대기
        msleep(1);
    }

    cap.release();
    stats.isConnected = false;
    emit statsUpdated(stats);
}

void applyFilters(cv::Mat &frame, int brightness, int contrast, int sharpen, int saturation) {
    // 1. 밝기(Brightness) 및 대비(Contrast) 조절
    // 공식: dst = src * (contrast/100 + 1) + brightness
    double alpha = (contrast / 100.0) + 1.0;
    frame.convertTo(frame, -1, alpha, brightness);

    // 2. 채도(Saturation) 조절
    if (saturation != 0) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        for (int r = 0; r < hsv.rows; ++r) {
            for (int c = 0; c < hsv.cols; ++c) {
                cv::Vec3b &pixel = hsv.at<cv::Vec3b>(r, c);
                // pixel[1] 이 Saturation(채도) 채널입니다.
                int newSat = pixel[1] + saturation;
                pixel[1] = cv::saturate_cast<uchar>(newSat);
            }
        }
        cv::cvtColor(hsv, frame, cv::COLOR_HSV2BGR);
    }

    // 3. 선명도(Sharpen) 조절 (언샤프 마스킹 필터)
    if (sharpen > 0) {
        cv::Mat blurred;
        // sharpen 값이 클수록 더 강한 블러와 차이를 둡니다.
        cv::GaussianBlur(frame, blurred, cv::Size(9, 9), sharpen / 10.0);
        // 원래 이미지와 블러된 이미지의 차이를 더해 선명하게 만듦
        cv::addWeighted(frame, 1.5, blurred, -0.5, 0, frame);
    }
}

void video_worker::setFilterMode(FilterMode mode) {
    m_filterMode = mode;
}

QImage video_worker::matToQImage(const cv::Mat& mat) {
    if (mat.type() == CV_8UC3) { // 8-bit, 3채널 (BGR)
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        return QImage((const unsigned char*)(rgbMat.data), rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888).copy();
    }
    else if (mat.type() == CV_8UC1) { // 8-bit, 1채널 (Grayscale)
        return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
    }
    return QImage();
}

void video_worker::setHomographyPoints(const QList<QPoint>& points) {
    m_homographySrcPoints.clear();
    for (const auto& pt : points) {
        m_homographySrcPoints.push_back(cv::Point2f(pt.x(), pt.y()));
    }
    m_updateHomography = true; // 새로운 좌표가 들어오면 H 행렬을 다시 계산하도록 플래그 켬
}
#ifndef VIDEOFILTERS_H
#define VIDEOFILTERS_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace videofilters {

// 화면 표시 전용 후처리. 분석(ArUco) 입력에는 적용하지 않는다.
inline void apply(cv::Mat &frame, int brightness, int contrast,
                  int sharpen, int saturation)
{
    if (frame.empty()) return;

    if (brightness != 0 || contrast != 0) {
        const double alpha = 1.0 + contrast / 100.0;
        frame.convertTo(frame, -1, alpha, brightness);
    }
    if (saturation != 0) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        channels[1].convertTo(channels[1], -1, 1.0, saturation);
        cv::merge(channels, hsv);
        cv::cvtColor(hsv, frame, cv::COLOR_HSV2BGR);
    }
    if (sharpen > 0) {
        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(9, 9), sharpen / 10.0);
        cv::addWeighted(frame, 1.5, blurred, -0.5, 0, frame);
    }
}

} // namespace videofilters

#endif // VIDEOFILTERS_H

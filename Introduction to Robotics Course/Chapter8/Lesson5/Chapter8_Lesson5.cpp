#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>

int main() {
    // (1) Acquire
    cv::Mat img = cv::imread("scene.png", cv::IMREAD_GRAYSCALE);

    // (2) Preprocess
    cv::Mat img_s;
    cv::GaussianBlur(img, img_s, cv::Size(5,5), 1.2);

    // (3) Feature extraction
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img_s, corners, 200, 0.01, 8);

    // (4) Weighted LS fusion
    Eigen::Vector2d x1(1.2, 0.5), x2(1.0, 0.9);
    Eigen::Matrix2d S1, S2;
    S1 << 0.04, 0.0, 0.0, 0.09;
    S2 << 0.16, 0.0, 0.0, 0.04;

    Eigen::Matrix2d W1 = S1.inverse();
    Eigen::Matrix2d W2 = S2.inverse();
    Eigen::Vector2d xf = (W1 + W2).inverse() * (W1 * x1 + W2 * x2);

    std::cout << "Fused estimate: " << xf.transpose() << std::endl;
    return 0;
}

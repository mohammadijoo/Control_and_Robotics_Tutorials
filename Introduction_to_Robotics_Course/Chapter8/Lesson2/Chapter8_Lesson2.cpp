#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    cv::Size patternSize(9, 6);
    float squareSize = 0.025f;

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // Prepare planar object points
    std::vector<cv::Point3f> objp;
    for (int y = 0; y < patternSize.height; ++y)
        for (int x = 0; x < patternSize.width; ++x)
            objp.emplace_back(x*squareSize, y*squareSize, 0.0f);

    std::vector<cv::String> images;
    cv::glob("calib_images/*.png", images);

    cv::Size imageSize;

    for (const auto& fname : images) {
        cv::Mat img = cv::imread(fname, cv::IMREAD_COLOR);
        imageSize = img.size();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool ok = cv::findChessboardCorners(gray, patternSize, corners);

        if (ok) {
            cv::cornerSubPix(
                gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 1e-6)
            );
            objpoints.push_back(objp);
            imgpoints.push_back(corners);
        }
    }

    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objpoints, imgpoints, imageSize, K, dist, rvecs, tvecs);

    std::cout << "RMS error: " << rms << std::endl;
    std::cout << "K=\n" << K << std::endl;
    std::cout << "dist=\n" << dist << std::endl;

    return 0;
}

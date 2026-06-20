// Chapter13_Lesson4.cpp
// Robustness utilities: affine brightness fit and blur metric (variance of Laplacian).
// Dependencies: OpenCV
//
// Build (example):
//   g++ -std=c++17 Chapter13_Lesson4.cpp `pkg-config --cflags --libs opencv4` -o lesson4
//
// Run:
//   ./lesson4 img1.png img2.png

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

static double varLaplacian(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mu, sigma;
    cv::meanStdDev(lap, mu, sigma);
    return sigma[0] * sigma[0];
}

static void affineBrightnessFit(const cv::Mat& I1, const cv::Mat& I2, double& a, double& b) {
    // Solve min_{a,b} || a*I1 + b - I2 ||^2 (least squares), grayscale images.
    CV_Assert(I1.size() == I2.size() && I1.type() == CV_8U && I2.type() == CV_8U);

    double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Sxy = 0.0;
    const int N = I1.rows * I1.cols;

    for (int y = 0; y < I1.rows; ++y) {
        const uchar* p1 = I1.ptr<uchar>(y);
        const uchar* p2 = I2.ptr<uchar>(y);
        for (int x = 0; x < I1.cols; ++x) {
            const double X = static_cast<double>(p1[x]);
            const double Y = static_cast<double>(p2[x]);
            Sx += X;   Sy += Y;
            Sxx += X * X;
            Sxy += X * Y;
        }
    }

    // Normal equations:
    // [Sxx Sx] [a] = [Sxy]
    // [Sx  N ] [b]   [Sy ]
    const double det = Sxx * N - Sx * Sx;
    if (std::abs(det) < 1e-12) {
        a = 1.0; b = 0.0; return;
    }
    a = (Sxy * N - Sy * Sx) / det;
    b = (Sxx * Sy - Sx * Sxy) / det;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " img1 img2\n";
        return 1;
    }

    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
        std::cerr << "Could not read images.\n";
        return 1;
    }
    cv::resize(img2, img2, img1.size());

    double a = 1.0, b = 0.0;
    affineBrightnessFit(img1, img2, a, b);

    const double s1 = varLaplacian(img1);
    const double s2 = varLaplacian(img2);

    std::cout << "Estimated affine brightness: a=" << a << " b=" << b << "\n";
    std::cout << "Blur score var(Laplacian): img1=" << s1 << " img2=" << s2 << "\n";
    return 0;
}

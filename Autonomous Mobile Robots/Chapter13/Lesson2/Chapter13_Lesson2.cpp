// Chapter13_Lesson2.cpp
/*
Autonomous Mobile Robots — Chapter 13, Lesson 2
Feature-Based vs Direct Methods (C++ / OpenCV)

Build (example):
  g++ -O2 -std=c++17 Chapter13_Lesson2.cpp -o lesson2 `pkg-config --cflags --libs opencv4`

Run:
  ./lesson2 --img1 frame1.png --img2 frame2.png --fx 525 --fy 525 --cx 319.5 --cy 239.5
*/
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

static cv::Mat loadGray(const std::string& path) {
    cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) throw std::runtime_error("Could not read image: " + path);
    return img;
}

static void featureBasedRelativePose(
    const cv::Mat& I1, const cv::Mat& I2, const cv::Mat& K,
    cv::Mat& R, cv::Mat& t_unit, cv::Mat& inlierMask,
    std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2
) {
    auto orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> k1, k2;
    cv::Mat d1, d2;
    orb->detectAndCompute(I1, cv::noArray(), k1, d1);
    orb->detectAndCompute(I2, cv::noArray(), k2, d2);
    if (d1.empty() || d2.empty()) throw std::runtime_error("Not enough descriptors.");

    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    bf.knnMatch(d1, d2, knn, 2);

    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for (auto& pair : knn) {
        if (pair.size() < 2) continue;
        if (pair[0].distance < 0.75f * pair[1].distance) good.push_back(pair[0]);
    }
    if (good.size() < 8) throw std::runtime_error("Not enough good matches.");

    pts1.clear(); pts2.clear();
    pts1.reserve(good.size()); pts2.reserve(good.size());
    for (auto& m : good) {
        pts1.push_back(k1[m.queryIdx].pt);
        pts2.push_back(k2[m.trainIdx].pt);
    }

    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, inlierMask);
    if (E.empty()) throw std::runtime_error("findEssentialMat failed.");

    cv::Mat t;
    cv::recoverPose(E, pts1, pts2, K, R, t, inlierMask);
    double nrm = cv::norm(t);
    t_unit = (nrm > 0) ? (t / nrm) : t;
}

static inline float huberWeight(float r, float delta) {
    float a = std::fabs(r);
    return (a <= delta) ? 1.0f : (delta / (a + 1e-12f));
}

static cv::Point2d directAlignTranslation(
    const cv::Mat& Iref_u8, const cv::Mat& Icur_u8,
    int levels = 4, int iters = 15, float huberDelta = 5.0f
) {
    std::vector<cv::Mat> pyrRef, pyrCur;
    pyrRef.reserve(levels); pyrCur.reserve(levels);
    pyrRef.push_back(Iref_u8);
    pyrCur.push_back(Icur_u8);
    for (int i = 1; i < levels; ++i) {
        cv::Mat r, c;
        cv::pyrDown(pyrRef.back(), r);
        cv::pyrDown(pyrCur.back(), c);
        pyrRef.push_back(r); pyrCur.push_back(c);
    }

    double u = 0.0, v = 0.0;

    for (int lvl = levels - 1; lvl >= 0; --lvl) {
        cv::Mat Rf, Cf;
        pyrRef[lvl].convertTo(Rf, CV_32F);
        pyrCur[lvl].convertTo(Cf, CV_32F);

        double scale = std::pow(2.0, lvl);
        double uL = u / scale;
        double vL = v / scale;

        for (int it = 0; it < iters; ++it) {
            cv::Mat M = (cv::Mat_<float>(2,3) << 1.f, 0.f, (float)uL, 0.f, 1.f, (float)vL);
            cv::Mat Cw;
            cv::warpAffine(Cf, Cw, M, Rf.size(), cv::INTER_LINEAR);

            cv::Mat r = Cw - Rf;

            cv::Mat Ix, Iy;
            cv::Sobel(Cw, Ix, CV_32F, 1, 0, 3);
            cv::Sobel(Cw, Iy, CV_32F, 0, 1, 3);

            const int b = 10;
            double A11=0, A12=0, A22=0, b1=0, b2=0;
            int count = 0;
            for (int y = b; y < r.rows - b; ++y) {
                const float* pr  = r.ptr<float>(y);
                const float* pgx = Ix.ptr<float>(y);
                const float* pgy = Iy.ptr<float>(y);
                for (int x = b; x < r.cols - b; ++x) {
                    float gx = pgx[x], gy = pgy[x];
                    float g2 = gx*gx + gy*gy;
                    if (g2 <= 1e-4f) continue;
                    float ri = pr[x];
                    float w  = huberWeight(ri, huberDelta);
                    A11 += w*gx*gx;
                    A12 += w*gx*gy;
                    A22 += w*gy*gy;
                    b1  += w*gx*ri;
                    b2  += w*gy*ri;
                    count++;
                }
            }
            if (count < 500) break;

            cv::Mat H = (cv::Mat_<double>(2,2) << A11, A12, A12, A22);
            cv::Mat g = (cv::Mat_<double>(2,1) << b1, b2);
            double lam = 1e-3 * (H.at<double>(0,0) + H.at<double>(1,1) + 1e-12);
            H += lam * cv::Mat::eye(2,2,CV_64F);

            cv::Mat duv;
            if (!cv::solve(H, -g, duv, cv::DECOMP_CHOLESKY)) break;

            uL += duv.at<double>(0,0);
            vL += duv.at<double>(1,0);

            if (cv::norm(duv) < 1e-3) break;
        }

        u = uL * scale;
        v = vL * scale;
    }

    return cv::Point2d(u, v);
}

int main(int argc, char** argv) {
    const cv::String keys =
        "{help h | | show help }"
        "{img1  | | path to reference image }"
        "{img2  | | path to current image }"
        "{fx    |525| fx }"
        "{fy    |525| fy }"
        "{cx    |319.5| cx }"
        "{cy    |239.5| cy }"
        "{no_vis|0| disable visualization }";

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help") || !parser.has("img1") || !parser.has("img2")) {
        std::cout << "Usage: ./lesson2 --img1 a.png --img2 b.png [--fx ... --fy ... --cx ... --cy ...]\n";
        return 0;
    }

    std::string img1 = parser.get<cv::String>("img1");
    std::string img2 = parser.get<cv::String>("img2");
    double fx = parser.get<double>("fx");
    double fy = parser.get<double>("fy");
    double cx = parser.get<double>("cx");
    double cy = parser.get<double>("cy");
    bool noVis = parser.get<int>("no_vis") != 0;

    cv::Mat I1 = loadGray(img1);
    cv::Mat I2 = loadGray(img2);

    cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    std::cout << "=== Feature-based (ORB + Essential + recoverPose) ===\n";
    cv::Mat R, t_unit, inliers;
    std::vector<cv::Point2f> pts1, pts2;
    featureBasedRelativePose(I1, I2, K, R, t_unit, inliers, pts1, pts2);
    std::cout << "R=\n" << R << "\n";
    std::cout << "t (unit norm)=\n" << t_unit.t() << " (monocular scale unknown)\n";

    std::cout << "\n=== Direct (translation-only photometric alignment) ===\n";
    cv::Point2d uv = directAlignTranslation(I1, I2);
    std::cout << "Estimated pixel translation: u=" << uv.x << ", v=" << uv.y << "\n";

    if (!noVis) {
        // Visualize direct alignment
        cv::Mat M = (cv::Mat_<float>(2,3) << 1.f, 0.f, (float)-uv.x, 0.f, 1.f, (float)-uv.y);
        cv::Mat I2w;
        cv::warpAffine(I2, I2w, M, I1.size(), cv::INTER_LINEAR);
        cv::Mat diff; cv::absdiff(I1, I2w, diff);
        cv::imshow("abs(I1 - warp(I2))", diff);
        cv::waitKey(0);
    }

    return 0;
}

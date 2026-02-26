// Chapter13_Lesson1.cpp
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 13 - Visual and Visual–Inertial SLAM (AMR Focus)
Lesson 1  - Visual Odometry for Mobile Robots

Minimal feature-based monocular VO in C++ (OpenCV):
  - ORB features
  - Descriptor matching (ratio test)
  - Essential matrix (RANSAC)
  - Pose recovery + chaining into a trajectory

Build (example):
  g++ -O2 -std=c++17 Chapter13_Lesson1.cpp `pkg-config --cflags --libs opencv4` -o vo_cpp

Run:
  ./vo_cpp --images path/to/frames --fx 525 --fy 525 --cx 319.5 --cy 239.5

Notes:
  - Monocular VO has unknown scale unless you fuse an external scale (wheel odom) or use stereo/RGB-D.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>

struct Intrinsics {
    double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5;
    cv::Mat K() const {
        return (cv::Mat_<double>(3,3) << fx, 0.0, cx,
                                         0.0, fy, cy,
                                         0.0, 0.0, 1.0);
    }
};

static std::vector<std::string> list_images(const std::string& folder) {
    std::vector<std::string> files;
    std::vector<std::string> patterns = {"/*.png","/*.jpg","/*.jpeg","/*.bmp"};
    for (const auto& p : patterns) {
        std::vector<cv::String> tmp;
        cv::glob(folder + p, tmp, false);
        for (const auto& s : tmp) files.push_back((std::string)s);
    }
    std::sort(files.begin(), files.end());
    return files;
}

static std::vector<cv::DMatch> ratio_match(const cv::Mat& d1, const cv::Mat& d2, double ratio=0.75) {
    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    bf.knnMatch(d1, d2, knn, 2);
    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for (const auto& pair : knn) {
        if (pair.size() < 2) continue;
        if (pair[0].distance < ratio * pair[1].distance) good.push_back(pair[0]);
    }
    return good;
}

static cv::Matx44d make_T(const cv::Mat& R, const cv::Mat& t, double scale=1.0) {
    cv::Matx44d T = cv::Matx44d::eye();
    for (int r=0; r<3; ++r) for (int c=0; c<3; ++c) T(r,c) = R.at<double>(r,c);
    T(0,3) = scale * t.at<double>(0);
    T(1,3) = scale * t.at<double>(1);
    T(2,3) = scale * t.at<double>(2);
    return T;
}

static cv::Vec3d camera_center_world(const cv::Matx44d& T_cw) {
    cv::Matx33d R;
    cv::Vec3d t(T_cw(0,3), T_cw(1,3), T_cw(2,3));
    for (int r=0; r<3; ++r) for (int c=0; c<3; ++c) R(r,c) = T_cw(r,c);
    cv::Vec3d Cw = -(R.t() * t);
    return Cw;
}

int main(int argc, char** argv) {
    const cv::String keys =
        "{help h ? |      | help }"
        "{images   |      | folder of frames }"
        "{fx       |525.0 | fx }"
        "{fy       |525.0 | fy }"
        "{cx       |319.5 | cx }"
        "{cy       |239.5 | cy }"
        "{max_frames|0    | process only first N frames (0=all)}";
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help") || !parser.has("images")) {
        std::cout << "Usage: vo_cpp --images <folder> [--fx ...]\n";
        return 0;
    }

    Intrinsics intr;
    intr.fx = parser.get<double>("fx");
    intr.fy = parser.get<double>("fy");
    intr.cx = parser.get<double>("cx");
    intr.cy = parser.get<double>("cy");
    int max_frames = parser.get<int>("max_frames");

    std::string folder = parser.get<std::string>("images");
    auto files = list_images(folder);
    if (files.size() < 2) {
        std::cerr << "Need at least 2 images in folder.\n";
        return 1;
    }
    if (max_frames > 1 && (size_t)max_frames < files.size()) files.resize((size_t)max_frames);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);
    cv::Mat img0 = cv::imread(files[0], cv::IMREAD_GRAYSCALE);
    if (img0.empty()) { std::cerr << "Cannot read: " << files[0] << "\n"; return 1; }
    std::vector<cv::KeyPoint> kp0;
    cv::Mat des0;
    orb->detectAndCompute(img0, cv::noArray(), kp0, des0);

    cv::Mat K = intr.K();

    cv::Matx44d T_cw = cv::Matx44d::eye();
    std::vector<cv::Vec3d> traj;
    traj.push_back(camera_center_world(T_cw));

    for (size_t i=1; i<files.size(); ++i) {
        cv::Mat img1 = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
        if (img1.empty()) { std::cerr << "Cannot read: " << files[i] << "\n"; break; }
        std::vector<cv::KeyPoint> kp1;
        cv::Mat des1;
        orb->detectAndCompute(img1, cv::noArray(), kp1, des1);

        if (des0.empty() || des1.empty() || kp0.size() < 20 || kp1.size() < 20) {
            std::cerr << "[WARN] low features at frame " << i << "\n";
            kp0 = kp1; des0 = des1.clone();
            continue;
        }

        auto matches = ratio_match(des0, des1, 0.75);
        if (matches.size() < 20) {
            std::cerr << "[WARN] low matches at frame " << i << "\n";
            kp0 = kp1; des0 = des1.clone();
            continue;
        }

        std::vector<cv::Point2f> p0, p1;
        p0.reserve(matches.size());
        p1.reserve(matches.size());
        for (const auto& m : matches) {
            p0.push_back(kp0[m.queryIdx].pt);
            p1.push_back(kp1[m.trainIdx].pt);
        }

        cv::Mat inliers;
        cv::Mat E = cv::findEssentialMat(p0, p1, K, cv::RANSAC, 0.999, 1.0, inliers);
        if (E.empty()) {
            std::cerr << "[WARN] E failed at frame " << i << "\n";
            kp0 = kp1; des0 = des1.clone();
            continue;
        }

        cv::Mat R, t;
        cv::recoverPose(E, p0, p1, K, R, t, inliers);

        cv::Matx44d T_12 = make_T(R, t, 1.0); // unit scale
        T_cw = T_12 * T_cw;

        traj.push_back(camera_center_world(T_cw));
        kp0 = kp1; des0 = des1.clone();

        if (i % 10 == 0 || i == files.size()-1) {
            auto C = traj.back();
            std::cout << "[INFO] frame " << i << "/" << (files.size()-1)
                      << " C_w = " << C[0] << ", " << C[1] << ", " << C[2] << "\n";
        }
    }

    std::ofstream out(folder + "/vo_trajectory_xyz.csv");
    out << "x,y,z\n";
    for (const auto& p : traj) out << p[0] << "," << p[1] << "," << p[2] << "\n";
    out.close();
    std::cout << "[DONE] trajectory saved to: " << folder << "/vo_trajectory_xyz.csv\n";
    return 0;
}

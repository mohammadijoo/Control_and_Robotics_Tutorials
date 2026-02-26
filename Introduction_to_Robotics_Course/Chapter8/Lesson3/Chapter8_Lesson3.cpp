#include <opencv2/opencv.hpp>
#include <iostream>
int main(){
    cv::Mat img = cv::imread("scene.png", cv::IMREAD_GRAYSCALE);

    // Edges
    cv::Mat Ix, Iy, G;
    cv::Sobel(img, Ix, CV_64F, 1, 0, 3);
    cv::Sobel(img, Iy, CV_64F, 0, 1, 3);
    cv::magnitude(Ix, Iy, G);
    cv::Mat edges = G > 50;

    // Corners
    cv::Mat harris;
    cv::cornerHarris(img, harris, 2, 3, 0.04);
    std::vector<cv::Point> corners;
    for(int y=0;y<harris.rows;y++)
      for(int x=0;x<harris.cols;x++)
        if(harris.at<float>(y,x) > 0.01f*harris.at<float>(0,0))
          corners.push_back(cv::Point(x,y));

    // Blobs
    auto detector = cv::SimpleBlobDetector::create();
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    std::cout << "corners=" << corners.size()
              << " blobs=" << keypoints.size() << std::endl;
    return 0;
}

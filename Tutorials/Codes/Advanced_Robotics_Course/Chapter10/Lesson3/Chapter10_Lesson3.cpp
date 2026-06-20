#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class SegmentationNode {
public:
    SegmentationNode(const std::string& onnx_path,
                     int num_classes)
        : it_(nh_), num_classes_(num_classes)
    {
        net_ = cv::dnn::readNetFromONNX(onnx_path);
        sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                             &SegmentationNode::imageCallback, this);
        pub_ = it_.advertise("/segmentation/mask", 1);
    }

    void spin() {
        ros::spin();
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        cv::Mat blob = cv::dnn::blobFromImage(
            img, 1.0 / 255.0, cv::Size(320, 240),
            cv::Scalar(0, 0, 0), true, false
        );

        net_.setInput(blob);
        cv::Mat scores = net_.forward(); // shape: (1, C, H, W)

        int C = num_classes_;
        int H = scores.size[2];
        int W = scores.size[3];

        cv::Mat mask(H, W, CV_8UC1);
        const float* data = reinterpret_cast<const float*>(scores.data);

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int best_class = 0;
                float best_score = data[y * W * C + x * C + 0];
                for (int c = 1; c < C; ++c) {
                    float val = data[y * W * C + x * C + c];
                    if (val > best_score) {
                        best_score = val;
                        best_class = c;
                    }
                }
                mask.at<unsigned char>(y, x) = static_cast<unsigned char>(best_class);
            }
        }

        cv::Mat mask_resized;
        cv::resize(mask, mask_resized, img.size(), 0, 0, cv::INTER_NEAREST);

        sensor_msgs::ImagePtr out_msg =
            cv_bridge::CvImage(msg->header, "mono8", mask_resized).toImageMsg();
        pub_.publish(out_msg);
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    cv::dnn::Net net_;
    int num_classes_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "semantic_segmentation_node_cpp");
    std::string onnx_path = "/path/to/model.onnx";
    int num_classes = 4;
    SegmentationNode node(onnx_path, num_classes);
    node.spin();
    return 0;
}
      

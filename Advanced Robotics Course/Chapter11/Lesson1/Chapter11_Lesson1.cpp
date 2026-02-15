#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <fstream>

struct Sample {
    double t;
    std::vector<double> q;
    std::vector<double> u;
};

class DemoLoggerNode {
public:
    DemoLoggerNode(ros::NodeHandle& nh)
        : start_time_(ros::Time::now()),
          have_last_cmd_(false) {
        joint_sub_ = nh.subscribe("/joint_states", 1,
                                  &DemoLoggerNode::jointCallback, this);
        cmd_sub_   = nh.subscribe("/teleop_cmd", 1,
                                  &DemoLoggerNode::cmdCallback, this);
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        double t = (msg->header.stamp - start_time_).toSec();
        std::vector<double> q = msg->position;

        std::vector<double> u;
        if (have_last_cmd_) {
            u = last_cmd_;
        } else {
            u.assign(q.size(), 0.0);
        }

        samples_.push_back({t, q, u});
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Example: map twist (vx, vy) to planar base velocity command
        last_cmd_.clear();
        last_cmd_.push_back(msg->linear.x);
        last_cmd_.push_back(msg->linear.y);
        have_last_cmd_ = true;
    }

    void saveCsv(const std::string& filename) const {
        std::ofstream file(filename.c_str());
        for (const auto& s : samples_) {
            file << s.t;
            for (double qi : s.q) file << "," << qi;
            for (double ui : s.u) file << "," << ui;
            file << "\n";
        }
        file.close();
    }

private:
    ros::Subscriber joint_sub_;
    ros::Subscriber cmd_sub_;
    ros::Time start_time_;
    bool have_last_cmd_;
    std::vector<double> last_cmd_;
    std::vector<Sample> samples_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_logger");
    ros::NodeHandle nh;

    DemoLoggerNode node(nh);
    ros::spin();

    node.saveCsv("teleop_demo.csv");
    return 0;
}
      

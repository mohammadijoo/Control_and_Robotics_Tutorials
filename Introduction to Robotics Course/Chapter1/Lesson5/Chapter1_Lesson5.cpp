
#include <iostream>
#include <Eigen/Dense>   // core robotics linear algebra

int main() {
  Eigen::Vector3d q(0.2, -0.4, 0.1);
  Eigen::Matrix<double,2,3> J;
  J << 1.0, 0.2, 0.0,
       0.0, 1.0, 0.3;
  Eigen::Vector3d qd(0.5, 0.0, -0.2);
  Eigen::Vector2d yd = J * qd;

  std::cout << "q = " << q.transpose() << std::endl;
  std::cout << "yd = " << yd.transpose() << std::endl;

  // Later chapters: rclcpp (ROS2), MoveIt, Pinocchio, Drake, Gazebo plugins.
  return 0;
}
      
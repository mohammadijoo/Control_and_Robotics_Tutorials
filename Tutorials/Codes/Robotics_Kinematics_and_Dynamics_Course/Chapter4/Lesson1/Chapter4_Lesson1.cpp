#include <iostream>
#include <vector>
#include <Eigen/Dense>

enum class JointType { REVOLUTE, PRISMATIC, FIXED };

struct Joint {
  JointType type;
  Eigen::Vector3d axis;   // axis in parent link frame
  double offset;          // constant offset
};

struct Link {
  std::string name;
  double length;
  Joint joint;
};

Eigen::Matrix4d rotZ(double theta) {
  double c = std::cos(theta), s = std::sin(theta);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0,0) = c;  T(0,1) = -s;
  T(1,0) = s;  T(1,1) =  c;
  return T;
}

Eigen::Matrix4d transX(double d) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0,3) = d;
  return T;
}

std::vector<Eigen::Matrix4d>
forwardKinematics(const std::vector<Link>& links,
                  const Eigen::VectorXd& q) {
  std::vector<Eigen::Matrix4d> Ts;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); // base
  Ts.push_back(T);

  for (std::size_t i = 0; i < links.size(); ++i) {
    const Link& L = links[i];
    double qi = q[i];
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

    if (L.joint.type == JointType::REVOLUTE) {
      double theta = L.joint.offset + qi;
      A = rotZ(theta) * transX(L.length);
    } else if (L.joint.type == JointType::PRISMATIC) {
      double d = L.joint.offset + qi;
      A = transX(d);
    }
    // FIXED: A is identity

    T = T * A;
    Ts.push_back(T);
  }
  return Ts;
}

int main() {
  std::vector<Link> links;
  links.push_back({"L1", 1.0, {JointType::REVOLUTE, {0,0,1}, 0.0}});
  links.push_back({"L2", 0.7, {JointType::REVOLUTE, {0,0,1}, 0.0}});

  Eigen::VectorXd q(2);
  q << M_PI / 4.0, -M_PI / 6.0;

  auto Ts = forwardKinematics(links, q);
  std::cout << "End-effector transform:\n" << Ts.back() << std::endl;
  return 0;
}
      

#include <iostream>
#include <vector>
#include <functional>
#include <Eigen/Dense>

class SerialChain {
public:
    using Transform = Eigen::Matrix4d;
    using JointTF = std::function<Transform(double)>;

    explicit SerialChain(int n_links)
        : n_links_(n_links),
          parent_(n_links),
          joint_tf_(n_links) {
        parent_[0] = -1;
        for (int i = 1; i < n_links_; ++i) {
            parent_[i] = i - 1;
        }
    }

    void setJointTransform(int i, JointTF f) {
        if (i <= 0 || i >= n_links_) {
            throw std::runtime_error("Invalid link index");
        }
        joint_tf_[i] = std::move(f);
    }

    std::vector<Transform> forwardTransforms(const std::vector<double>& q) const {
        std::vector<Transform> T(n_links_, Transform::Identity());
        for (int i = 1; i < n_links_; ++i) {
            int p = parent_[i];
            Transform T_pi = joint_tf_[i](q[i - 1]);
            T[i] = T[p] * T_pi;
        }
        return T;
    }

private:
    int n_links_;
    std::vector<int> parent_;
    std::vector<JointTF> joint_tf_;
};

// Example planar revolute joint
SerialChain::Transform planarR(double theta, double a) {
    SerialChain::Transform T = SerialChain::Transform::Identity();
    double c = std::cos(theta), s = std::sin(theta);
    T(0,0) = c;  T(0,1) = -s;
    T(1,0) = s;  T(1,1) =  c;
    T(0,3) = a;
    return T;
}

int main() {
    SerialChain chain(4); // links 0..3
    std::vector<double> a = {1.0, 1.0, 1.0};

    for (int i = 1; i <= 3; ++i) {
        chain.setJointTransform(i,
            [ai = a[i - 1]](double q_i) {
                return planarR(q_i, ai);
            }
        );
    }

    std::vector<double> q = {0.2, 0.4, -0.3};
    auto T = chain.forwardTransforms(q);
    std::cout << "End-effector position: "
              << T.back()(0,3) << " "
              << T.back()(1,3) << " "
              << T.back()(2,3) << std::endl;
    return 0;
}
      

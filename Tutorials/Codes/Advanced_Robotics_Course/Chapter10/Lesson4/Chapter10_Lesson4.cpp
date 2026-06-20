#include <Eigen/Dense>
#include <vector>
#include <cmath>

using Feature = Eigen::VectorXf;

float sigmoid(float z) {
    return 1.0f / (1.0f + std::exp(-z));
}

class LogisticAffordance {
public:
    explicit LogisticAffordance(int d)
        : theta_(d)
    {
        theta_.setZero();
    }

    float score(const Feature& phi) const {
        float z = theta_.dot(phi);
        return sigmoid(z);  // probability of success in (0,1)
    }

    void trainBatch(const std::vector<Feature>& features,
                    const std::vector<int>& labels,
                    float lr,
                    float reg)
    {
        Eigen::VectorXf grad(theta_.size());
        grad.setZero();
        const std::size_t N = features.size();

        for (std::size_t i = 0; i != N; ++i) {
            float p = score(features[i]);
            float diff = static_cast<float>(labels[i]) - p;
            grad += -diff * features[i];  // negative log-likelihood
        }

        grad /= static_cast<float>(N);
        grad += reg * theta_;

        theta_ -= lr * grad;
    }

    Eigen::VectorXf& theta() { return theta_; }
    const Eigen::VectorXf& theta() const { return theta_; }

private:
    Eigen::VectorXf theta_;
};

// In a ROS node, features would be computed from a point cloud
// in the gripper coordinate frame obtained via TF and robot kinematics.
      

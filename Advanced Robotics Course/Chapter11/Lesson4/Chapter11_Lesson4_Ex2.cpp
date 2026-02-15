#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Segment {
  int start;
  int end;  // inclusive
};

double segmentVariance(const MatrixXd& phi, int i, int j) {
  // phi: T x d
  int L = j - i + 1;
  MatrixXd seg = phi.block(i, 0, L, phi.cols());
  VectorXd mu = seg.colwise().mean();
  MatrixXd centered = seg.rowwise() - mu.transpose();
  double var = (centered.array() * centered.array()).sum() / (L * phi.cols());
  return var;
}

std::vector<Segment> greedySegment(const MatrixXd& phi, double var_threshold, int max_len) {
  std::vector<Segment> segments;
  int T = static_cast<int>(phi.rows());
  int i = 0;
  while (i < T) {
    int j = std::min(i + max_len - 1, T - 1);
    // shrink j until variance is below threshold
    while (j > i) {
      double var = segmentVariance(phi, i, j);
      if (var < var_threshold) break;
      --j;
    }
    Segment s{i, j};
    segments.push_back(s);
    i = j + 1;
  }
  return segments;
}

// In a ROS node, phi could be filled from sensor_msgs/JointState callbacks,
// and the resulting segments published as a custom message or used to
// parameterize movement primitives in a manipulation pipeline.
      

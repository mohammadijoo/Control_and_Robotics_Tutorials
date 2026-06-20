#include <Eigen/Dense>
#include <vector>

using Transform = Eigen::Matrix4d;
using SpatialMatrix = Eigen::Matrix<double,6,6>;
using SpatialVector = Eigen::Matrix<double,6,1>;

struct Link
{
  int parent;                 // index of parent link
  SpatialVector S;            // motion subspace
  Transform T_parent_to_i0;   // transform at q_i = 0
};

class KinematicTree
{
public:
  explicit KinematicTree(const std::vector<Link>& links)
    : links_(links)
  {}

  void forwardKinematics(const Transform& T_base,
                         const SpatialVector& v_base,
                         const Eigen::VectorXd& q,
                         std::vector<Transform>& T_world,
                         std::vector<SpatialVector>& v_world) const
  {
    const int n_links = static_cast<int>(links_.size());
    T_world.resize(n_links);
    v_world.resize(n_links);

    // base
    T_world[0] = T_base;
    v_world[0] = v_base;

    for (int i = 1; i < n_links; ++i)
    {
      const Link& link = links_[i];
      int parent = link.parent;

      Transform T_joint = expSE3(link.S, q(i-1)); // user-defined exponential
      T_world[i] = T_world[parent] * link.T_parent_to_i0 * T_joint;

      SpatialMatrix X_i_parent = adjoint(link.T_parent_to_i0 * T_joint);
      v_world[i] = X_i_parent * v_world[parent]; // + link.S * qdot(i-1) when needed
    }
  }

private:
  std::vector<Link> links_;
};
      

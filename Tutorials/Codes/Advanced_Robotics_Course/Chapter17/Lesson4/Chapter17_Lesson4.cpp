#include <iostream>
#include <Eigen/Dense>

int main() {
    const int obs_dim = 128;
    const int act_dim = 6;

    // Policy matrix W of size (act_dim x obs_dim), learned offline
    Eigen::MatrixXd W(act_dim, obs_dim);
    W.setRandom(); // replace with loaded weights

    // Example observation feature vector (encoded deformable state)
    Eigen::VectorXd obs(obs_dim);
    obs.setRandom(); // replace with real features

    // Compute action u = W * obs
    Eigen::VectorXd u = W * obs;

    std::cout << "Commanded velocity:" << std::endl;
    std::cout << u.transpose() << std::endl;

    return 0;
}
      

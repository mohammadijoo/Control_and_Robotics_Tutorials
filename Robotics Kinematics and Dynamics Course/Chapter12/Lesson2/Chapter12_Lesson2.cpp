#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct LinkDynamics {
    Eigen::Vector3d F;   // inertial force
    Eigen::Vector3d N;   // inertial moment
    Eigen::Matrix3d R_ip1_i; // ^i R_(i+1)
    Eigen::Vector3d p_i; // from frame i to i+1 in frame i
    Eigen::Vector3d z;   // joint axis
    char joint_type;     // 'R' or 'P'
};

void newtonEulerBackward(const std::vector<LinkDynamics>& links,
                         const Eigen::Vector3d& f_tip,
                         const Eigen::Vector3d& n_tip,
                         std::vector<Eigen::Vector3d>& f_list,
                         std::vector<Eigen::Vector3d>& n_list,
                         std::vector<double>& tau_list)
{
    int n = static_cast<int>(links.size());
    f_list.assign(n, Eigen::Vector3d::Zero());
    n_list.assign(n, Eigen::Vector3d::Zero());
    tau_list.assign(n, 0.0);

    Eigen::Vector3d f_next = f_tip;
    Eigen::Vector3d n_next = n_tip;

    for (int i = n - 1; i >= 0; --i) {
        const auto& L = links[i];

        Eigen::Vector3d f_child_i = L.R_ip1_i * f_next;
        Eigen::Vector3d n_child_i = L.R_ip1_i * n_next
                                  + L.p_i.cross(f_child_i);

        Eigen::Vector3d f_i = L.F + f_child_i;
        Eigen::Vector3d n_i = L.N + n_child_i;

        f_list[i] = f_i;
        n_list[i] = n_i;

        double tau_i;
        if (L.joint_type == 'R') {
            tau_i = L.z.dot(n_i);
        } else { // 'P'
            tau_i = L.z.dot(f_i);
        }
        tau_list[i] = tau_i;

        f_next = f_i;
        n_next = n_i;
    }
}

int main() {
    std::vector<LinkDynamics> links(2);
    // Initialize with some toy data (in a real application, fill from model)
    links[0].F = Eigen::Vector3d(1.0, 0.0, 0.0);
    links[0].N = Eigen::Vector3d(0.0, 0.0, 0.2);
    links[0].R_ip1_i = Eigen::Matrix3d::Identity();
    links[0].p_i = Eigen::Vector3d(0.5, 0.0, 0.0);
    links[0].z = Eigen::Vector3d(0.0, 0.0, 1.0);
    links[0].joint_type = 'R';

    links[1].F = Eigen::Vector3d(0.5, 0.0, 0.0);
    links[1].N = Eigen::Vector3d(0.0, 0.0, 0.1);
    links[1].R_ip1_i = Eigen::Matrix3d::Identity();
    links[1].p_i = Eigen::Vector3d(0.3, 0.0, 0.0);
    links[1].z = Eigen::Vector3d(0.0, 0.0, 1.0);
    links[1].joint_type = 'R';

    Eigen::Vector3d f_tip = Eigen::Vector3d::Zero();
    Eigen::Vector3d n_tip = Eigen::Vector3d::Zero();

    std::vector<Eigen::Vector3d> f_list, n_list;
    std::vector<double> tau_list;

    newtonEulerBackward(links, f_tip, n_tip, f_list, n_list, tau_list);

    std::cout << "Joint torques: ";
    for (double tau : tau_list) {
        std::cout << tau << " ";
    }
    std::cout << std::endl;
    return 0;
}
      

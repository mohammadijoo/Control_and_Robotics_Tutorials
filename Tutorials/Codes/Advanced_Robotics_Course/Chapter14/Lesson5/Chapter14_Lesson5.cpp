#include <Eigen/Dense>
#include <vector>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd planarGraspMatrix(const std::vector<Eigen::Vector2d> &contacts)
{
    int m = static_cast<int>(contacts.size());
    MatrixXd G(3, 2 * m);
    G.setZero();
    for (int i = 0; i < m; ++i) {
        double x = contacts;
        double y = contacts;
        int col = 2 * i;
        G(0, col)     = 1.0;
        G(1, col + 1) = 1.0;
        G(2, col)     = -y;
        G(2, col + 1) = x;
    }
    return G;
}

int main()
{
    // Example contacts
    std::vector<Eigen::Vector2d> contacts;
    contacts.emplace_back(0.5, 0.0);
    contacts.emplace_back(-0.5, 0.0);

    MatrixXd G = planarGraspMatrix(contacts);
    int m = static_cast<int>(contacts.size());
    MatrixXd W = MatrixXd::Identity(2 * m, 2 * m);

    double m_obj = 2.0;
    double g = 9.81;
    VectorXd w_des(3);
    w_des << 0.0, m_obj * g, 0.0;

    MatrixXd GWG = G * W.inverse() * G.transpose();
    VectorXd lambda_vec = GWG.ldlt().solve(w_des);
    VectorXd f_c_star = W.inverse() * G.transpose() * lambda_vec;

    std::cout << "Optimal forces f_c* = " << f_c_star.transpose() << std::endl;

    // ---------- Greedy task allocation (for small problems) ----------
    const int nRobots = 3;
    const int nTasks  = 3;
    double C[nRobots][nTasks] = {
        {4.0, 2.0, 3.5},
        {2.5, 3.0, 2.0},
        {3.0, 4.0, 1.5}
    };

    std::vector<int> taskOfRobot(nRobots, -1);
    std::vector<bool> taskTaken(nTasks, false);

    for (int i = 0; i < nRobots; ++i) {
        double best = 1e9;
        int bestTask = -1;
        for (int j = 0; j < nTasks; ++j) {
            if (taskTaken[j]) {
                continue;
            }
            if (C[i][j] < best) {
                best = C[i][j];
                bestTask = j;
            }
        }
        if (bestTask != -1) {
            taskOfRobot[i] = bestTask;
            taskTaken[bestTask] = true;
        }
    }

    for (int i = 0; i < nRobots; ++i) {
        std::cout << "robot " << i
                  << " -> task " << taskOfRobot[i]
                  << std::endl;
    }

    return 0;
}
      

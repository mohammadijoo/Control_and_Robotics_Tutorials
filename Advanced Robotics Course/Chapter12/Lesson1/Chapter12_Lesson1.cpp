#include <Eigen/Dense>
#include <cmath>

struct RobotState {
    Eigen::VectorXd x; // [q; dq]
};

struct RobotAction {
    Eigen::VectorXd u; // torques or commands
};

class RobotMDP {
public:
    RobotMDP(int n_dof, int m_act, double dt)
        : n_(n_dof), m_(m_act), dt_(dt)
    {
        Q_ = Eigen::MatrixXd::Identity(2 * n_, 2 * n_);
        R_ = 0.01 * Eigen::MatrixXd::Identity(m_, m_);
        q_limit_ = Eigen::VectorXd::Constant(n_, M_PI);
        u_limit_ = Eigen::VectorXd::Constant(m_, 10.0);
    }

    RobotState dynamics(const RobotState& s, const RobotAction& a) const {
        Eigen::VectorXd q = s.x.head(n_);
        Eigen::VectorXd dq = s.x.tail(n_);

        Eigen::VectorXd u = a.u;
        u = u.cwiseMax(-u_limit_).cwiseMin(u_limit_);

        Eigen::VectorXd ddq = u; // placeholder for M(q)^{-1} B u

        Eigen::VectorXd q_next = q + dt_ * dq;
        Eigen::VectorXd dq_next = dq + dt_ * ddq;
        q_next = q_next.cwiseMax(-q_limit_).cwiseMin(q_limit_);

        RobotState s_next;
        s_next.x.resize(2 * n_);
        s_next.x << q_next, dq_next;
        return s_next;
    }

    double reward(const RobotState& s, const RobotAction& a,
                  const RobotState* s_des = nullptr) const {
        Eigen::VectorXd x_des;
        if (s_des) {
            x_des = s_des->x;
        } else {
            x_des = Eigen::VectorXd::Zero(2 * n_);
        }

        Eigen::VectorXd dx = s.x - x_des;
        double cost = dx.transpose() * Q_ * dx
                    + a.u.transpose() * R_ * a.u;
        return -cost;
    }

    // One MDP step: (s, a) -> (s_next, r)
    void step(const RobotState& s, const RobotAction& a,
              RobotState& s_next, double& r, bool& done) const {
        s_next = dynamics(s, a);
        r = reward(s, a, nullptr);

        done = (s_next.x.norm() > 1e3);
    }

private:
    int n_, m_;
    double dt_;
    Eigen::MatrixXd Q_, R_;
    Eigen::VectorXd q_limit_, u_limit_;
};
      

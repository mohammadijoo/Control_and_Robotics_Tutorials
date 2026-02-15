#include <Eigen/Dense>
#include <vector>
#include <random>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct Transition {
    VectorXd s;
    VectorXd a;
    double r;
    VectorXd s_next;
    bool done;
};

class PlanarManipDPG {
public:
    PlanarManipDPG(int state_dim, int action_dim)
        : state_dim_(state_dim),
          action_dim_(action_dim),
          W_(MatrixXd::Zero(action_dim, state_dim)),
          critic_P_(MatrixXd::Identity(state_dim, state_dim)),
          gamma_(0.99),
          alpha_actor_(1e-3),
          alpha_critic_(1e-3) {}

    VectorXd policy(const VectorXd& s) const {
        // Deterministic linear policy: a = W s
        return W_ * s;
    }

    double critic(const VectorXd& s) const {
        // Quadratic critic V(s) = s' P s
        return 0.5 * s.transpose() * critic_P_ * s;
    }

    void update(const std::vector<Transition>& batch) {
        // Critic update: TD(0) on V(s) ~= s' P s
        for (const auto& tr : batch) {
            double v_s = critic(tr.s);
            double v_next = critic(tr.s_next);
            double td_target = tr.r + (tr.done ? 0.0 : gamma_ * v_next);
            double td_error = td_target - v_s;

            // Gradient wrt P: dV/dP = 0.5 * (s s' + s s')
            MatrixXd grad_P = -td_error * (tr.s * tr.s.transpose());
            critic_P_ -= alpha_critic_ * grad_P;
        }

        // Actor update: DPG-style using critic gradient approximated by P
        for (const auto& tr : batch) {
            VectorXd a = policy(tr.s);
            // Here we fake Q gradient as proportional to -a and -s for illustration.
            // In practice, use a separate Q(s,a) critic.
            VectorXd grad_mu = a;  // d(mu)/dW * dQ/da; here simplified
            MatrixXd grad_W = grad_mu * tr.s.transpose();
            W_ += alpha_actor_ * grad_W;
        }
    }

private:
    int state_dim_;
    int action_dim_;
    MatrixXd W_;
    MatrixXd critic_P_;
    double gamma_;
    double alpha_actor_;
    double alpha_critic_;
};

// Pseudo-code for rollout and training loop
int main() {
    const int state_dim = 6;
    const int action_dim = 2;
    PlanarManipDPG agent(state_dim, action_dim);

    std::mt19937 rng(0);
    for (int episode = 0; episode < 500; ++episode) {
        VectorXd s = VectorXd::Zero(state_dim); // reset from simulator
        bool done = false;
        std::vector<Transition> traj;
        while (!done) {
            VectorXd a = agent.policy(s);
            // simulate one step: stepDynamics(s, a, s_next, r, done)
            VectorXd s_next(state_dim);
            double r = 0.0;
            stepDynamics(s, a, s_next, r, done); // user-implemented dynamics

            traj.push_back({s, a, r, s_next, done});
            s = s_next;
        }
        agent.update(traj);
    }
    return 0;
}
      

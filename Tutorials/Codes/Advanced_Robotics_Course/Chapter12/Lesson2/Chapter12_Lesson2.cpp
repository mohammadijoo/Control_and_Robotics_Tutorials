#include <Eigen/Dense>
#include <random>
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct Actor {
    MatrixXd W;   // weights: act_dim x obs_dim
    VectorXd b;   // bias: act_dim
    double log_std;

    Actor(int obs_dim, int act_dim)
        : W(MatrixXd::Zero(act_dim, obs_dim)),
          b(VectorXd::Zero(act_dim)),
          log_std(std::log(0.1)) {}

    VectorXd mean(const VectorXd& s) const {
        return W * s + b;
    }

    VectorXd sample(const VectorXd& s, double& logp,
                    std::mt19937& gen) const {
        VectorXd mu = mean(s);
        double std = std::exp(log_std);
        std::normal_distribution<double> dist(0.0, std);
        VectorXd eps(mu.size());
        for (int i = 0; i < mu.size(); ++i) {
            eps(i) = dist(gen);
        }
        VectorXd a = mu + eps;
        // log probability
        double log_det = mu.size() * std::log(std);
        double quad = (a - mu).squaredNorm() / (2.0 * std * std);
        logp = -0.5 * mu.size() * std::log(2.0 * M_PI)
               - log_det - quad;
        return a;
    }

    // gradient of log pi wrt W for linear Gaussian (no squashing)
    MatrixXd grad_logp_W(const VectorXd& s,
                         const VectorXd& a) const {
        double std = std::exp(log_std);
        VectorXd mu = mean(s);
        VectorXd diff = (a - mu) / (std * std);
        // outer product: diff * s'
        return diff * s.transpose();
    }

    VectorXd grad_logp_b(const VectorXd& s,
                         const VectorXd& a) const {
        double std = std::exp(log_std);
        VectorXd mu = mean(s);
        return (a - mu) / (std * std);
    }
};

struct Critic {
    VectorXd w;  // linear value: V(s) = w' s

    explicit Critic(int obs_dim) : w(VectorXd::Zero(obs_dim)) {}

    double value(const VectorXd& s) const {
        return w.dot(s);
    }
};

int main() {
    const int obs_dim = 4;  // example: pendulum state
    const int act_dim = 1;
    Actor actor(obs_dim, act_dim);
    Critic critic(obs_dim);
    double alpha_actor = 1e-3;
    double alpha_critic = 5e-3;
    double gamma = 0.99;
    std::mt19937 gen(42);

    // Pseudo-code loop:
    // for each episode:
    //   rollout states, actions, rewards, next_states using physics engine
    std::vector<VectorXd> states, next_states, actions;
    std::vector<double> rewards, logps;

    // ... fill buffers from simulator ...

    // Online update
    for (size_t t = 0; t < states.size(); ++t) {
        const VectorXd& s = states[t];
        const VectorXd& a = actions[t];
        const VectorXd& s_next = next_states[t];
        double r = rewards[t];

        double V_s = critic.value(s);
        double V_next = critic.value(s_next);
        double delta = r + gamma * V_next - V_s;

        // critic update
        critic.w += alpha_critic * delta * s;

        // actor update
        MatrixXd grad_W = actor.grad_logp_W(s, a);
        VectorXd grad_b = actor.grad_logp_b(s, a);
        actor.W += alpha_actor * delta * grad_W;
        actor.b += alpha_actor * delta * grad_b;
    }

    return 0;
}
      

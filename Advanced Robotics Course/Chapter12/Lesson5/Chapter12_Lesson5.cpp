#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <random>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class SafeActorCritic {
public:
    SafeActorCritic(int obs_dim, int act_dim)
    : obs_dim_(obs_dim), act_dim_(act_dim),
      lambda_c_(0.0), lambda_lr_(1e-3)
    {
        // Initialize parameters (e.g., small random values)
        theta_ = VectorXd::Zero(num_params());
        w_r_ = VectorXd::Zero(num_value_params());
        w_c_ = VectorXd::Zero(num_value_params());
    }

    VectorXd act(const VectorXd &obs, double &logp_out) {
        // Gaussian policy: a = mu_theta(obs) + sigma * noise
        VectorXd phi = feature_actor(obs);
        VectorXd mu = W_policy_ * phi;
        double sigma = 0.1;  // fixed std for illustration

        VectorXd noise = VectorXd::NullaryExpr(act_dim_, [&](){
            return normal_(rng_);
        });
        VectorXd a = mu + sigma * noise;

        double logp = -0.5 * (noise.squaredNorm()
                     + act_dim_ * std::log(2.0 * M_PI * sigma * sigma));
        logp_out = logp;
        return a;
    }

    void update_batch(const std::vector<VectorXd> &obs_traj,
                      const std::vector<VectorXd> &act_traj,
                      const std::vector<double> &rew_traj,
                      const std::vector<double> &cost_traj,
                      double gamma)
    {
        // Compute discounted returns G_r, G_c
        int T = static_cast<int>(rew_traj.size());
        std::vector<double> G_r(T), G_c(T);
        double g_r = 0.0, g_c = 0.0;
        for (int t = T - 1; t >= 0; --t) {
            g_r = rew_traj[t] + gamma * g_r;
            g_c = cost_traj[t] + gamma * g_c;
            G_r[t] = g_r;
            G_c[t] = g_c;
        }

        // Simple gradient estimates (omitting baselines for brevity)
        VectorXd grad_theta = VectorXd::Zero(theta_.size());
        double avg_cost = 0.0;
        for (int t = 0; t < T; ++t) {
            VectorXd phi = feature_actor(obs_traj[t]);
            VectorXd mu = W_policy_ * phi;
            // score function grad wrt theta (here compressed as grad_mu)
            VectorXd grad_mu = phi;  // for linear policy mu = W * phi
            VectorXd a = act_traj[t];
            VectorXd diff = a - mu;
            double sigma = 0.1;

            double factor = (G_r[t] - lambda_c_ * G_c[t]) / (sigma * sigma);
            grad_theta += factor * kron(diff, grad_mu); // kron product

            avg_cost += cost_traj[t];
        }
        avg_cost /= static_cast<double>(T);

        // Gradient ascent on theta
        theta_ += 1e-3 * grad_theta;

        // Dual update
        lambda_c_ = std::max(0.0, lambda_c_ + lambda_lr_ *
                                       (avg_cost - cost_limit_));
    }

private:
    int obs_dim_, act_dim_;
    VectorXd theta_, w_r_, w_c_;
    MatrixXd W_policy_;
    double lambda_c_;
    double lambda_lr_;
    double cost_limit_ = 1.0;

    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<double> normal_{0.0, 1.0};

    int num_params() const { return obs_dim_ * act_dim_; }
    int num_value_params() const { return obs_dim_; }

    VectorXd feature_actor(const VectorXd &obs) const {
        // e.g., identity features
        return obs;
    }

    VectorXd kron(const VectorXd &a, const VectorXd &b) const {
        VectorXd out(a.size() * b.size());
        int k = 0;
        for (int i = 0; i < a.size(); ++i) {
            for (int j = 0; j < b.size(); ++j) {
                out(k++) = a(i) * b(j);
            }
        }
        return out;
    }
};
      

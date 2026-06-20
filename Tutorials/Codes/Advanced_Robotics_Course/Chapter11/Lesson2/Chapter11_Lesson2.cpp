#include <iostream>
#include <vector>
#include <Eigen/Dense>

using State = Eigen::Vector4d;   // [q1, q2, dq1, dq2]
using Action = Eigen::Vector2d;  // [tau1, tau2]

struct Sample {
    State s;
    Action a;
};

class LinearPolicy {
public:
    Eigen::Matrix<double, 2, 4> W;
    Action b;

    LinearPolicy() {
        W.setZero();
        b.setZero();
    }

    Action operator()(const State &s) const {
        return W * s + b;
    }
};

// Dummy expert PD controller
Action expertPolicy(const State &s) {
    Eigen::Vector2d q_des(0.5, -0.5);
    Eigen::Vector2d q = s.segment<2>(0);
    Eigen::Vector2d dq = s.segment<2>(2);
    double Kp = 5.0, Kd = 1.0;
    return Kp * (q_des - q) - Kd * dq;
}

// Simple discrete-time dynamics for illustration
State simulate(const State &s, const Action &a, double dt = 0.02) {
    Eigen::Vector2d q = s.segment<2>(0);
    Eigen::Vector2d dq = s.segment<2>(2);
    Eigen::Vector2d ddq = a; // placeholder
    Eigen::Vector2d dq_next = dq + ddq * dt;
    Eigen::Vector2d q_next = q + dq_next * dt;
    State s_next;
    s_next << q_next, dq_next;
    return s_next;
}

// Least-squares fit of linear policy: minimize sum ||W s + b - a||^2
void trainBC(LinearPolicy &pi, const std::vector<Sample> &data, int iters = 100, double lr = 1e-3) {
    for (int k = 0; k < iters; ++k) {
        Eigen::Matrix<double, 2, 4> gradW = Eigen::Matrix<double, 2, 4>::Zero();
        Action gradb = Action::Zero();
        for (const auto &sample : data) {
            Action pred = pi(sample.s);
            Action err = pred - sample.a;
            gradW += err * sample.s.transpose();
            gradb += err;
        }
        gradW /= static_cast<double>(data.size());
        gradb /= static_cast<double>(data.size());
        pi.W -= lr * gradW;
        pi.b -= lr * gradb;
    }
}

std::vector<Sample> collectExpertRollout(int T) {
    std::vector<Sample> traj;
    State s = State::Zero();
    for (int t = 0; t < T; ++t) {
        Action a = expertPolicy(s);
        traj.push_back({s, a});
        s = simulate(s, a);
    }
    return traj;
}

// DAgger training loop
void daggerTrain(int numIters = 5, int T = 200) {
    LinearPolicy pi;
    std::vector<Sample> dataset = collectExpertRollout(T);

    for (int i = 0; i < numIters; ++i) {
        trainBC(pi, dataset, 200, 1e-3);

        // Rollout current policy
        State s = State::Zero();
        std::vector<Sample> newSamples;
        for (int t = 0; t < T; ++t) {
            Action a_pi = pi(s);
            // Query expert at visited state s
            Action a_exp = expertPolicy(s);
            newSamples.push_back({s, a_exp});
            s = simulate(s, a_pi);
        }

        // Aggregate
        dataset.insert(dataset.end(), newSamples.begin(), newSamples.end());
        std::cout << "DAgger iter " << (i+1)
                  << ", dataset size = " << dataset.size() << std::endl;
    }
}

int main() {
    daggerTrain();
    return 0;
}
      

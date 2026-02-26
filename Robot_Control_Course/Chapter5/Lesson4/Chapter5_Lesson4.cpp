
#include <iostream>
#include <vector>

enum class Mode { Free, Contact };

struct State {
    std::vector<double> q;
    std::vector<double> v;
};

struct ContactInfo {
    bool active;
    double gap;        // signed distance
    double normalVel;  // end-effector normal velocity
};

class ImpactController {
public:
    ImpactController(double e) : e_(e), mode_(Mode::Free) {}

    void step(double t, double dt, State &x) {
        ContactInfo c = detectContact(x);
        if (mode_ == Mode::Free && c.gap <= 0.0 && c.normalVel < 0.0) {
            // impact event
            applyImpact(x, c);
            mode_ = Mode::Contact;
        } else if (mode_ == Mode::Contact && c.normalVel >= 0.0 && c.active == false) {
            // liftoff
            mode_ = Mode::Free;
        }

        std::vector<double> tau = computeControl(x, mode_);
        integrateDynamics(t, dt, x, tau);
    }

private:
    Mode mode_;
    double e_; // restitution

    ContactInfo detectContact(const State &x) {
        ContactInfo c{};
        // user-defined kinematics to compute gap and normal velocity
        // c.gap = phi(q); c.normalVel = dphi(q, v);
        return c;
    }

    void applyImpact(State &x, const ContactInfo &c) {
        // Use impact law: v_plus = v_minus - M^{-1} J^T * Lambda_n
        // Here we only sketch the structure; actual implementation requires
        // access to M(q) and J_n(q).
        // M * (v_plus - v_minus) = J_n^T * Lambda_n
        // dot_xn_minus = J_n * v_minus
        // Lambda_n = -(1+e) * dot_xn_minus / (J_n * M^{-1} * J_n^T)
    }

    std::vector<double> computeControl(const State &x, Mode mode) {
        // PD or model-based control with mode-dependent damping gains
        std::vector<double> tau(x.q.size(), 0.0);
        // ...
        return tau;
    }

    void integrateDynamics(double t, double dt, State &x,
                           const std::vector<double> &tau) {
        // Use your favourite integrator (Euler, Runge-Kutta, etc.)
        // M(q) * dv/dt + h(q, v) = tau + J_c^T f_c
    }
};

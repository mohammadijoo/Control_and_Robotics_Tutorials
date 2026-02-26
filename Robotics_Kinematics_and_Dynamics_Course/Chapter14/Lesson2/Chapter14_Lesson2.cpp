#include <iostream>

struct GearJointParameters {
    double Jm;   // motor inertia
    double Jg;   // gear inertia on motor side
    double JL;   // link inertia at joint
    double bm;   // motor viscous friction
    double bq;   // joint viscous friction
    double n;    // gear ratio theta_m = n * q
    double eta;  // efficiency
};

class GearJointDynamicModel {
public:
    GearJointDynamicModel(const GearJointParameters& p)
    : p_(p)
    {
        computeReflectedParameters();
    }

    // Compute qdd for given state and motor torque
    double acceleration(double q, double qdot, double tau_m, double tau_ext = 0.0) const {
        // J_eq * qdd + b_eq * qdot = n * eta * tau_m + tau_ext
        double rhs = p_.n * p_.eta * tau_m + tau_ext - b_eq_ * qdot;
        return rhs / J_eq_;
    }

    double J_eq() const { return J_eq_; }
    double b_eq() const { return b_eq_; }

private:
    GearJointParameters p_;
    double J_eq_;
    double b_eq_;

    void computeReflectedParameters() {
        J_eq_ = p_.n * p_.n * (p_.Jm + p_.Jg) + p_.JL;
        b_eq_ = p_.n * p_.n * p_.bm + p_.bq;
    }
};

int main() {
    GearJointParameters params;
    params.Jm = 0.002;
    params.Jg = 0.001;
    params.JL = 0.05;
    params.bm = 0.001;
    params.bq = 0.02;
    params.n  = 50.0;
    params.eta = 0.9;

    GearJointDynamicModel model(params);

    double q = 0.0;
    double qdot = 0.0;
    double dt = 0.001;

    for (int k = 0; k < 1000; ++k) {
        double tau_m = 1.0; // step motor torque
        double qdd = model.acceleration(q, qdot, tau_m);
        qdot += qdd * dt;
        q += qdot * dt;
    }

    std::cout << "Equivalent inertia (joint side): " << model.J_eq() << std::endl;
    std::cout << "Final joint angle: " << q << " rad" << std::endl;
    return 0;
}
      

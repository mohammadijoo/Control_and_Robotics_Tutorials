#include <random>
#include <array>

struct ArmParams {
    std::array<double, 2> link_masses;
    double contact_friction;
};

class PlanarArmEnvDR {
public:
    PlanarArmEnvDR()
    : gen_(std::random_device{}()),
      unif01_(0.0, 1.0)
    {
        mass_nominal_[0] = 1.0;
        mass_nominal_[1] = 1.0;
        friction_nominal_ = 0.5;
        mass_rel_range_ = 0.3;
        friction_rel_range_ = 0.5;
    }

    void reset() {
        sampleParameters();
        // reset simulator state, call engine API, etc.
        // engine_.setLinkMass(0, params_.link_masses[0]);
        // engine_.setLinkMass(1, params_.link_masses[1]);
        // engine_.setContactFriction(params_.contact_friction);
    }

    const ArmParams& currentParams() const { return params_; }

    // step(...) would call physics engine with params_

private:
    void sampleParameters() {
        for (int i = 0; i < 2; ++i) {
            double low = (1.0 - mass_rel_range_) * mass_nominal_[i];
            double high = (1.0 + mass_rel_range_) * mass_nominal_[i];
            params_.link_masses[i] = uniform(low, high);
        }
        double lowf = (1.0 - friction_rel_range_) * friction_nominal_;
        double highf = (1.0 + friction_rel_range_) * friction_nominal_;
        params_.contact_friction = uniform(lowf, highf);
    }

    double uniform(double a, double b) {
        return a + (b - a) * unif01_(gen_);
    }

    std::mt19937 gen_;
    std::uniform_real_distribution<double> unif01_;

    std::array<double, 2> mass_nominal_;
    double friction_nominal_;
    double mass_rel_range_;
    double friction_rel_range_;

    ArmParams params_;
};
      

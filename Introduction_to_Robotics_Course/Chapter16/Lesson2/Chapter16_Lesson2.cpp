#include <iostream>
#include <vector>
#include <string>
#include <limits>

struct Concept {
    std::string name;
    double cost;
    double mass;
    double energy;
    double tracking_error;
};

struct NormalizedConcept {
    std::string name;
    double z_cost;
    double z_mass;
    double z_energy;
    double z_error;
};

int main() {
    std::vector<Concept> concepts = {
        {"A_wheeled", 15.0, 40.0, 800.0, 1.5},
        {"B_tracked", 18.0, 50.0, 700.0, 1.2},
        {"C_legged",  25.0, 55.0, 950.0, 0.8}
    };

    const double MAX_MASS = 55.0;
    const double MAX_ENERGY = 1000.0;

    std::vector<Concept> feasible;
    for (auto &c : concepts) {
        if (c.mass <= MAX_MASS && c.energy <= MAX_ENERGY) {
            feasible.push_back(c);
        }
    }

    // Find min and max for each metric
    auto init_min = std::numeric_limits<double>::max();
    auto init_max = std::numeric_limits<double>::lowest();
    double cost_min = init_min, cost_max = init_max;
    double mass_min = init_min, mass_max = init_max;
    double energy_min = init_min, energy_max = init_max;
    double error_min = init_min, error_max = init_max;

    for (auto &c : feasible) {
        cost_min = std::min(cost_min, c.cost);
        cost_max = std::max(cost_max, c.cost);
        mass_min = std::min(mass_min, c.mass);
        mass_max = std::max(mass_max, c.mass);
        energy_min = std::min(energy_min, c.energy);
        energy_max = std::max(energy_max, c.energy);
        error_min = std::min(error_min, c.tracking_error);
        error_max = std::max(error_max, c.tracking_error);
    }

    auto norm = [](double x, double xmin, double xmax) {
        if (std::abs(xmax - xmin) < 1e-9) return 1.0;
        return (xmax - x) / (xmax - xmin);
    };

    std::vector<NormalizedConcept> Z;
    for (auto &c : feasible) {
        NormalizedConcept z{
            c.name,
            norm(c.cost,   cost_min,   cost_max),
            norm(c.mass,   mass_min,   mass_max),
            norm(c.energy, energy_min, energy_max),
            norm(c.tracking_error, error_min, error_max)
        };
        Z.push_back(z);
    }

    // weights: cost, mass, energy, tracking_error
    double w_cost = 0.3, w_mass = 0.2, w_energy = 0.2, w_error = 0.3;

    double best_U = -1e9;
    std::string best_name;

    for (auto &z : Z) {
        double U = w_cost * z.z_cost
                 + w_mass * z.z_mass
                 + w_energy * z.z_energy
                 + w_error * z.z_error;
        std::cout << z.name << " : U = " << U << std::endl;
        if (U > best_U) {
            best_U = U;
            best_name = z.name;
        }
    }

    std::cout << "Best concept: " << best_name
              << " with utility " << best_U << std::endl;

    return 0;
}
      

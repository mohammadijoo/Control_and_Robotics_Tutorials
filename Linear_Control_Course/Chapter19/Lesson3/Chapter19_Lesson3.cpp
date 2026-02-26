#include <cmath>
#include <iostream>

struct LeadParams {
    double alpha;
    double tau;
    double Kc;
};

LeadParams designLeadParameters(double phiMaxDeg,
                                double wcDes,
                                double magC0G_at_wc) {
    // phi_max in radians
    double phiMax = phiMaxDeg * M_PI / 180.0;
    double s = std::sin(phiMax);

    // alpha = (1 - sin(phi_max))/(1 + sin(phi_max))
    double alpha = (1.0 - s) / (1.0 + s);

    // tau = 1/(wc * sqrt(alpha))
    double tau = 1.0 / (wcDes * std::sqrt(alpha));

    // Kc to enforce |Kc C0(j wc) G(j wc)| = 1
    // Input argument magC0G_at_wc is |C0(j wc) G(j wc)|
    double Kc = 1.0 / magC0G_at_wc;

    return {alpha, tau, Kc};
}

// Example usage in a robotics-oriented controller
int main() {
    double phiMaxDeg = 45.0;
    double wcDes = 4.0;
    // Suppose we have computed |C0(j wc) G(j wc)| via offline analysis:
    double magC0G_at_wc = 0.146; // Example from worked example

    LeadParams p = designLeadParameters(phiMaxDeg, wcDes, magC0G_at_wc);

    std::cout << "alpha = " << p.alpha << "\n";
    std::cout << "tau   = " << p.tau   << "\n";
    std::cout << "Kc    = " << p.Kc    << "\n";

    // In a real robot joint controller, you would implement
    // u(s)/e(s) = Kc (tau s + 1)/(alpha tau s + 1)
    // via a discretized difference equation in your control loop.
    return 0;
}

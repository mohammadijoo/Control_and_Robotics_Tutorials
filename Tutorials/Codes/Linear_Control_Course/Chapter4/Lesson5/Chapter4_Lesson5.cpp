#include <vector>
#include <cmath>

// Compute Delta(s) given loop gains and combinations of non-touching loops.
// nonTouching[k] is a vector of indices of loops forming one non-touching set.
double computeDelta(
    const std::vector<double>& L,
    const std::vector< std::vector<int> >& nonTouching)
{
    double Delta = 1.0;

    // Subtract sum of individual loops
    for (double Lj : L) {
        Delta -= Lj;
    }

    // Add/subtract higher-order products according to number of loops in each set
    for (const auto& idxSet : nonTouching) {
        double prod = 1.0;
        for (int idx : idxSet) {
            prod *= L.at(static_cast<std::size_t>(idx));
        }
        std::size_t m = idxSet.size();
        double sign = std::pow(-1.0, static_cast<int>(m));
        Delta += sign * prod;
    }

    return Delta;
}

// Mason gain for single input-single output (scalar gains)
double masonGain(
    const std::vector<double>& P,
    const std::vector<double>& L,
    const std::vector< std::vector<int> >& nonTouching,
    const std::vector< std::vector<int> >& touchingMask)
{
    double Delta = computeDelta(L, nonTouching);
    double num = 0.0;

    // For each forward path k, build Deltak by removing loops that touch path k.
    for (std::size_t k = 0; k < P.size(); ++k) {
        std::vector<double> Lk;
        std::vector< std::vector<int> > nonTouchingK;

        // Keep only loops not touching path k
        for (std::size_t j = 0; j < L.size(); ++j) {
            if (!touchingMask[k][j]) {
                Lk.push_back(L[j]);
            }
        }

        // For simplicity, we ignore recomputing non-touching sets for Lk here.
        // For many robotics SFGs with only a few loops, this can be enumerated explicitly.

        double Delta_k = computeDelta(Lk, nonTouchingK);
        num += P[k] * Delta_k;
    }

    return num / Delta;
}

// Example: simple feedback P1 = G, one loop L1 = -G*H (no non-touching pairs)
int main()
{
    double G = 10.0;
    double H = 0.5;
    std::vector<double> P = {G};
    std::vector<double> L = {-G * H};
    std::vector< std::vector<int> > nonTouching;     // empty
    std::vector< std::vector<int> > touchingMask = {{1}}; // loop 0 touches path 0

    double T = masonGain(P, L, nonTouching, touchingMask);
    // T should equal G / (1 + G*H)
    return 0;
}

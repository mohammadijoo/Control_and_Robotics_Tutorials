#include <vector>
#include <cmath>
#include <cassert>

class CSpace {
public:
    CSpace(const std::vector<double>& lower_bounds,
           const std::vector<double>& upper_bounds,
           const std::vector<bool>& periodic)
        : lower_(lower_bounds),
          upper_(upper_bounds),
          periodic_(periodic) {
        assert(lower_.size() == upper_.size());
        assert(lower_.size() == periodic_.size());
        n_ = static_cast<std::size_t>(lower_.size());
    }

    std::vector<double> wrap(const std::vector<double>& q) const {
        std::vector<double> res = q;
        for (std::size_t i = 0; i < n_; ++i) {
            if (periodic_[i]) {
                double x = res[i];
                // wrap to (-pi, pi]
                x = std::fmod(x + M_PI, 2.0 * M_PI);
                if (x < 0.0) x += 2.0 * M_PI;
                res[i] = x - M_PI;
            }
        }
        return res;
    }

    std::vector<double> shortestDifference(const std::vector<double>& q_from,
                                             const std::vector<double>& q_to) const {
        std::vector<double> a = wrap(q_from);
        std::vector<double> b = wrap(q_to);
        std::vector<double> diff(n_);
        for (std::size_t i = 0; i < n_; ++i) {
            double d = b[i] - a[i];
            if (periodic_[i]) {
                if (d > M_PI) {
                    d -= 2.0 * M_PI;
                } else if (d < -M_PI) {
                    d += 2.0 * M_PI;
                }
            }
            diff[i] = d;
        }
        return diff;
    }

    std::vector<std::vector<double> >
    interpolate(const std::vector<double>& q_start,
                const std::vector<double>& q_goal,
                std::size_t num_samples) const {
        std::vector<double> qs = wrap(q_start);
        std::vector<double> diff = shortestDifference(qs, q_goal);
        std::vector<std::vector<double> > path;
        path.reserve(num_samples);
        for (std::size_t k = 0; k < num_samples; ++k) {
            double alpha = 0.0;
            if (num_samples > 1) {
                alpha = static_cast<double>(k) /
                        static_cast<double>(num_samples - 1);
            }
            std::vector<double> q(n_);
            for (std::size_t i = 0; i < n_; ++i) {
                q[i] = qs[i] + alpha * diff[i];
            }
            q = wrap(q);
            path.push_back(q);
        }
        return path;
    }

private:
    std::vector<double> lower_, upper_;
    std::vector<bool> periodic_;
    std::size_t n_;
};

// Example usage (2R planar arm):
// CSpace cspace({-M_PI, -M_PI}, {M_PI, M_PI}, {true, true});
// auto path = cspace.interpolate({0.0, 0.0}, {M_PI, -M_PI}, 5);
      

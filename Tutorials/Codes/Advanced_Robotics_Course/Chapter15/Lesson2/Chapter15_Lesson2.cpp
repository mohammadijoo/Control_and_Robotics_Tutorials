#include <vector>
#include <cmath>
#include <random>

struct Vec2 {
    double x{0.0}, y{0.0};
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
};

double norm(const Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

class Swarm2D {
public:
    enum Behavior { AGGREGATE, DISPERSE };

    Swarm2D(std::size_t N, double dt, Behavior b)
        : N_(N), dt_(dt), behavior_(b), x_(N) {
        std::mt19937 gen(42);
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        for (auto& p : x_) {
            p.x = uni(gen);
            p.y = uni(gen);
        }
    }

    void step() {
        if (behavior_ == AGGREGATE) stepAggregate();
        else stepDispersion();
    }

    const std::vector<Vec2>& positions() const { return x_; }

private:
    std::size_t N_;
    double dt_;
    Behavior behavior_;
    std::vector<Vec2> x_;
    double R_ = 0.4;
    double k_agg_ = 1.0;
    double k_disp_ = 0.5;
    double d_des_ = 0.2;

    void neighbors(std::size_t i,
                   std::vector<std::size_t>& idx,
                   std::vector<Vec2>& diffs,
                   std::vector<double>& dists) const {
        idx.clear();
        diffs.clear();
        dists.clear();
        for (std::size_t j = 0; j < N_; ++j) {
            if (j == i) continue;
            Vec2 d = x_[j] - x_[i];
            double r = norm(d);
            if (r > 0.0 && r < R_) {
                idx.push_back(j);
                diffs.push_back(d);
                dists.push_back(r);
            }
        }
    }

    void stepAggregate() {
        std::vector<Vec2> u(N_);
        std::vector<std::size_t> idx;
        std::vector<Vec2> diffs;
        std::vector<double> dists;
        for (std::size_t i = 0; i < N_; ++i) {
            neighbors(i, idx, diffs, dists);
            Vec2 ui(0.0, 0.0);
            for (const auto& d : diffs) ui += d;
            u[i] = ui * (-k_agg_);
        }
        for (std::size_t i = 0; i < N_; ++i) {
            x_[i] += u[i] * dt_;
        }
    }

    void stepDispersion() {
        std::vector<Vec2> u(N_);
        std::vector<std::size_t> idx;
        std::vector<Vec2> diffs;
        std::vector<double> dists;
        for (std::size_t i = 0; i < N_; ++i) {
            neighbors(i, idx, diffs, dists);
            Vec2 ui(0.0, 0.0);
            for (std::size_t k = 0; k < idx.size(); ++k) {
                const Vec2& d = diffs[k];
                double r = dists[k];
                double r2 = r * r;
                double phi_prime = (r2 - d_des_ * d_des_) * r;
                Vec2 dir(d.x / r, d.y / r);
                ui += dir * (-phi_prime);
            }
            u[i] = ui * k_disp_;
        }
        for (std::size_t i = 0; i < N_; ++i) {
            x_[i] += u[i] * dt_;
        }
    }
};
      

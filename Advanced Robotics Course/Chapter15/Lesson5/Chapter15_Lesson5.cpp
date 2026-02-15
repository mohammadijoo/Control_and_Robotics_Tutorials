#include <vector>
#include <cmath>
#include <iostream>

struct Vec2 {
    double x{0.0}, y{0.0};
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2 operator+(const Vec2& o) const { return Vec2{x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return Vec2{x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return Vec2{x * s, y * s}; }
};

double norm2(const Vec2& v) { return v.x * v.x + v.y * v.y; }

int main() {
    const int N = 50;
    const double h = 0.05;
    const double R = 2.0;
    const double w_att = 0.2;
    const double w_rep = 0.05;
    const double d_min = 0.5;
    const double v_max = 0.5;
    const int steps = 500;

    std::vector<Vec2> pos(N);
    std::vector<Vec2> vel(N);

    // Simple initialization: agents on a circle
    for (int i = 0; i < N; ++i) {
        double theta = 2.0 * M_PI * static_cast<double>(i) / N;
        pos[i] = Vec2{5.0 * std::cos(theta), 5.0 * std::sin(theta)};
    }

    for (int k = 0; k < steps; ++k) {
        // Zero velocities
        for (int i = 0; i < N; ++i) {
            vel[i] = Vec2{0.0, 0.0};
        }

        // Compute local controls
        for (int i = 0; i < N; ++i) {
            Vec2 p_i = pos[i];
            Vec2 att{0.0, 0.0};
            Vec2 rep{0.0, 0.0};
            for (int j = 0; j < N; ++j) {
                if (j == i) continue;
                Vec2 diff = Vec2{pos[j].x - p_i.x, pos[j].y - p_i.y};
                double d2 = norm2(diff);
                if (d2 <= R * R) {
                    att += diff;
                    if (d2 > 1e-12) {
                        double d = std::sqrt(d2);
                        if (d <= d_min) {
                            // Short-range repulsion
                            double scale = -1.0 / (d * d * d);
                            rep += diff * scale;
                        }
                    }
                }
            }
            Vec2 u_i = att * w_att + rep * w_rep;
            vel[i] = u_i;
        }

        // Saturate velocities and integrate
        for (int i = 0; i < N; ++i) {
            double speed2 = norm2(vel[i]);
            if (speed2 > v_max * v_max) {
                double speed = std::sqrt(speed2);
                double scale = v_max / speed;
                vel[i] = vel[i] * scale;
            }
            pos[i].x += h * vel[i].x;
            pos[i].y += h * vel[i].y;
        }
    }

    // Output final positions (for plotting elsewhere)
    for (int i = 0; i < N; ++i) {
        std::cout << pos[i].x << " " << pos[i].y << "\n";
    }

    return 0;
}
      

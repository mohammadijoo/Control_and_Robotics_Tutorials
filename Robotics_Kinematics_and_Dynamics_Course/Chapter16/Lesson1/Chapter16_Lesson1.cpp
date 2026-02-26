#include <iostream>
#include <array>
#include <cmath>

struct Vec2 {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double phi;
};

using Vec3 = std::array<double, 3>;

static const std::array<Vec2, 3> B = { {
    {0.0, 0.0},
    {1.0, 0.0},
    {0.5, std::sqrt(3.0) / 2.0}
} };

static const double rp = 0.2;
static const std::array<Vec2, 3> P = { {
    { rp * 1.0,                 rp * 0.0 },
    { rp * (-0.5),              rp * (std::sqrt(3.0) / 2.0) },
    { rp * (-0.5),              rp * (-std::sqrt(3.0) / 2.0) }
} };

inline Vec2 rot2(const Vec2 &v, double phi) {
    double c = std::cos(phi);
    double s = std::sin(phi);
    return { c * v.x - s * v.y,
             s * v.x + c * v.y };
}

Vec3 ik_3rpr(const Pose &pose) {
    Vec3 L{};
    for (int i = 0; i < 3; ++i) {
        Vec2 Pi = P[i];
        Vec2 Pi_base = rot2(Pi, pose.phi);
        Vec2 ci { pose.x + Pi_base.x, pose.y + Pi_base.y };
        Vec2 pi { ci.x - B[i].x, ci.y - B[i].y };
        L[i] = std::sqrt(pi.x * pi.x + pi.y * pi.y);
    }
    return L;
}

Vec3 fk_residual(const Pose &pose, const Vec3 &L) {
    Vec3 r{};
    for (int i = 0; i < 3; ++i) {
        Vec2 Pi = P[i];
        Vec2 Pi_base = rot2(Pi, pose.phi);
        Vec2 ci { pose.x + Pi_base.x, pose.y + Pi_base.y };
        Vec2 pi { ci.x - B[i].x, ci.y - B[i].y };
        double norm2 = pi.x * pi.x + pi.y * pi.y;
        r[i] = norm2 - L[i] * L[i];
    }
    return r;
}

std::array<Vec3, 3> fk_jacobian(const Pose &pose) {
    std::array<Vec3, 3> J{};
    double c = std::cos(pose.phi);
    double s = std::sin(pose.phi);
    // derivative of rotation w.r.t. phi
    for (int i = 0; i < 3; ++i) {
        Vec2 Pi = P[i];
        Vec2 Pi_base { c * Pi.x - s * Pi.y,
                       s * Pi.x + c * Pi.y };
        Vec2 ci { pose.x + Pi_base.x, pose.y + Pi_base.y };
        Vec2 pi { ci.x - B[i].x, ci.y - B[i].y };
        // R_phi * Pi
        Vec2 dpi_dphi { -s * Pi.x - c * Pi.y,
                         c * Pi.x - s * Pi.y };

        double dphidx   = 2.0 * pi.x;
        double dphidy   = 2.0 * pi.y;
        double dphidphi = 2.0 * (pi.x * dpi_dphi.x + pi.y * dpi_dphi.y);

        J[i][0] = dphidx;
        J[i][1] = dphidy;
        J[i][2] = dphidphi;
    }
    return J;
}

// Solve 3x3 linear system J * dx = rhs using Cramer's rule (for simplicity)
bool solve3x3(const std::array<Vec3, 3> &J, const Vec3 &rhs, Vec3 &dx) {
    auto det3 = [](const std::array<Vec3, 3> &M) {
        return M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
             - M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
             + M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);
    };

    double detJ = det3(J);
    if (std::fabs(detJ) < 1e-12) {
        return false;
    }

    std::array<Vec3, 3> Mx = J;
    std::array<Vec3, 3> My = J;
    std::array<Vec3, 3> Mphi = J;

    for (int i = 0; i < 3; ++i) {
        Mx[i][0]   = rhs[i];
        My[i][1]   = rhs[i];
        Mphi[i][2] = rhs[i];
    }

    dx[0] = det3(Mx)   / detJ;
    dx[1] = det3(My)   / detJ;
    dx[2] = det3(Mphi) / detJ;
    return true;
}

bool fk_3rpr_newton(const Vec3 &L, Pose &pose, int max_iter = 50, double tol = 1e-10) {
    for (int k = 0; k < max_iter; ++k) {
        Vec3 r = fk_residual(pose, L);
        double norm_r = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
        if (norm_r < tol) {
            return true;
        }
        std::array<Vec3, 3> J = fk_jacobian(pose);
        Vec3 rhs { -r[0], -r[1], -r[2] };
        Vec3 dx{};
        if (!solve3x3(J, rhs, dx)) {
            return false; // singular
        }
        pose.x   += dx[0];
        pose.y   += dx[1];
        pose.phi += dx[2];
    }
    return false;
}

int main() {
    Pose pose_true {0.2, 0.1, 0.3};
    Vec3 L = ik_3rpr(pose_true);

    Pose pose_guess {0.0, 0.0, 0.0};
    bool ok = fk_3rpr_newton(L, pose_guess);
    std::cout << "Converged: " << ok << "\n";
    std::cout << "True pose: "
              << pose_true.x << ", "
              << pose_true.y << ", "
              << pose_true.phi << "\n";
    std::cout << "FK pose:   "
              << pose_guess.x << ", "
              << pose_guess.y << ", "
              << pose_guess.phi << "\n";
    return 0;
}
      

// Chapter3_Lesson3.cpp
// Autonomous Mobile Robots — Chapter 3 Lesson 3
// Feasible Path Families for Car-Like Robots (Dubins from scratch; RS via OMPL optional)
//
// Build (no OMPL):
//   g++ -O2 -std=c++17 Chapter3_Lesson3.cpp -o Chapter3_Lesson3
//
// Run:
//   ./Chapter3_Lesson3
//
// Output:
//   dubins_path.csv (x,y,theta)

#include <cmath>
#include <cfloat>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

static constexpr double PI = 3.14159265358979323846;

static double mod2pi(double a) {
    a = std::fmod(a, 2.0 * PI);
    if (a < 0.0) a += 2.0 * PI;
    return a;
}

static double angdiff(double a, double b) {
    // minimal signed angle difference a-b in [-pi, pi)
    double d = std::fmod(a - b + PI, 2.0 * PI);
    if (d < 0.0) d += 2.0 * PI;
    return d - PI;
}

struct Pose2 {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct DubinsPath {
    std::string type;              // LSL, RSR, ...
    std::string seg_types;         // length 3 string, e.g., "LSL"
    std::tuple<double,double,double> params; // (t,p,q) in normalized units; turns are angles [rad]
    double R_min{1.0};

    double length() const {
        auto [t,p,q] = params;
        return R_min * (t + p + q);
    }
};

static Pose2 dubinsSegmentUnit(const Pose2& s, char segType, double segLen) {
    Pose2 p = s;
    if (segType == 'S') {
        p.x += segLen * std::cos(p.theta);
        p.y += segLen * std::sin(p.theta);
        return p;
    }
    if (segType == 'L') {
        p.x += std::sin(p.theta + segLen) - std::sin(p.theta);
        p.y += -std::cos(p.theta + segLen) + std::cos(p.theta);
        p.theta = mod2pi(p.theta + segLen);
        return p;
    }
    if (segType == 'R') {
        p.x += -std::sin(p.theta - segLen) + std::sin(p.theta);
        p.y += std::cos(p.theta - segLen) - std::cos(p.theta);
        p.theta = mod2pi(p.theta - segLen);
        return p;
    }
    throw std::runtime_error("Unknown segType");
}

static double endpointError(double alpha, double d, double beta,
                            const std::string& segTypes,
                            const std::tuple<double,double,double>& prm) {
    Pose2 p{0.0, 0.0, alpha};
    auto [t, s, q] = prm;
    p = dubinsSegmentUnit(p, segTypes[0], t);
    p = dubinsSegmentUnit(p, segTypes[1], s);
    p = dubinsSegmentUnit(p, segTypes[2], q);
    return std::hypot(p.x - d, p.y) + std::fabs(angdiff(p.theta, beta));
}

// ---- Candidate formulas (standard Dubins closed forms) ----

static bool LSL(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    double sa = std::sin(alpha), sb = std::sin(beta);
    double ca = std::cos(alpha), cb = std::cos(beta);
    double cab = std::cos(alpha - beta);

    double tmp0 = d + sa - sb;
    double p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(sa - sb);
    if (p2 < 0.0) return false;
    double p = std::sqrt(p2);
    double tmp1 = std::atan2(cb - ca, tmp0);
    double t = mod2pi(-alpha + tmp1);
    double q = mod2pi(beta - tmp1);
    out = {t,p,q};
    return true;
}

static bool RSR(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    double sa = std::sin(alpha), sb = std::sin(beta);
    double ca = std::cos(alpha), cb = std::cos(beta);
    double cab = std::cos(alpha - beta);

    double tmp0 = d - sa + sb;
    double p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(-sa + sb);
    if (p2 < 0.0) return false;
    double p = std::sqrt(p2);
    double tmp1 = std::atan2(ca - cb, tmp0);
    double t = mod2pi(alpha - tmp1);
    double q = mod2pi(-beta + tmp1);
    out = {t,p,q};
    return true;
}

static bool LSR(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    double sa = std::sin(alpha), sb = std::sin(beta);
    double ca = std::cos(alpha), cb = std::cos(beta);
    double cab = std::cos(alpha - beta);

    double p2 = -2.0 + d*d + 2.0*cab + 2.0*d*(sa + sb);
    if (p2 < 0.0) return false;
    double p = std::sqrt(p2);
    double tmp0 = std::atan2(-ca - cb, d + sa + sb);
    double tmp1 = std::atan2(-2.0, p);
    double t = mod2pi(-alpha + tmp0 - tmp1);
    double q = mod2pi(-beta + tmp0 - tmp1);
    out = {t,p,q};
    return true;
}

static bool RSL(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    double sa = std::sin(alpha), sb = std::sin(beta);
    double ca = std::cos(alpha), cb = std::cos(beta);
    double cab = std::cos(alpha - beta);

    double p2 = -2.0 + d*d + 2.0*cab - 2.0*d*(sa + sb);
    if (p2 < 0.0) return false;
    double p = std::sqrt(p2);
    double tmp0 = std::atan2(ca + cb, d - sa - sb);
    double tmp1 = std::atan2(2.0, p);
    double t = mod2pi(alpha - tmp0 + tmp1);
    double q = mod2pi(beta - tmp0 + tmp1);
    out = {t,p,q};
    return true;
}

static bool RLR(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    double sa = std::sin(alpha), sb = std::sin(beta);
    double ca = std::cos(alpha), cb = std::cos(beta);
    double cab = std::cos(alpha - beta);

    double tmp0 = (6.0 - d*d + 2.0*cab + 2.0*d*(sa - sb)) / 8.0;
    if (std::fabs(tmp0) > 1.0) return false;
    double p = mod2pi(2.0*PI - std::acos(tmp0));
    double tmp1 = std::atan2(ca - cb, d - sa + sb);
    double t = mod2pi(alpha - tmp1 + p/2.0);
    double q = mod2pi(alpha - beta - t + p);
    out = {t,p,q};
    return true;
}

static bool LRL(double alpha, double beta, double d, std::tuple<double,double,double>& out) {
    // symmetry: LRL(alpha,beta,d) = RLR(-alpha,-beta,d)
    return RLR(mod2pi(-alpha), mod2pi(-beta), d, out);
}

// ---- Main solver ----

static DubinsPath dubinsShortest(const Pose2& q0, const Pose2& q1, double R_min) {
    if (R_min <= 0.0) throw std::runtime_error("R_min must be positive");

    // frame of q0
    double dx = q1.x - q0.x;
    double dy = q1.y - q0.y;
    double c0 = std::cos(q0.theta), s0 = std::sin(q0.theta);

    double x = (c0*dx + s0*dy) / R_min;
    double y = (-s0*dx + c0*dy) / R_min;
    double phi = mod2pi(q1.theta - q0.theta);

    double d = std::hypot(x, y);
    double theta = (d > 0.0) ? std::atan2(y, x) : 0.0;

    double alpha = mod2pi(-theta);
    double beta  = mod2pi(phi - theta);

    struct Cand { std::string type; std::string seg; bool (*fn)(double,double,double,std::tuple<double,double,double>&); };
    std::vector<Cand> cands = {
        {"LSL","LSL", LSL},
        {"RSR","RSR", RSR},
        {"LSR","LSR", LSR},
        {"RSL","RSL", RSL},
        {"RLR","RLR", RLR},
        {"LRL","LRL", LRL}
    };

    double bestLen = DBL_MAX;
    DubinsPath best;

    for (const auto& c : cands) {
        std::tuple<double,double,double> prm;
        if (!c.fn(alpha, beta, d, prm)) continue;
        double err = endpointError(alpha, d, beta, c.seg, prm);
        if (err > 1e-6) continue;
        double L = R_min * (std::get<0>(prm) + std::get<1>(prm) + std::get<2>(prm));
        if (L < bestLen) {
            bestLen = L;
            best = DubinsPath{c.type, c.seg, prm, R_min};
        }
    }

    if (bestLen >= DBL_MAX/2.0) throw std::runtime_error("No feasible Dubins path found");
    return best;
}

static std::vector<Pose2> samplePath(const Pose2& q0, const DubinsPath& path, double step) {
    auto [t,p,q] = path.params;
    std::vector<std::pair<char,double>> segs = {
        {path.seg_types[0], t},
        {path.seg_types[1], p},
        {path.seg_types[2], q}
    };

    std::vector<Pose2> pts;
    pts.push_back(q0);

    Pose2 cur = q0;

    for (auto [st, sl] : segs) {
        if (st == 'S') {
            double L = sl * path.R_min;
            int n = std::max(1, (int)std::ceil(L / step));
            double ds = L / n;
            for (int i = 0; i < n; ++i) {
                cur.x += ds * std::cos(cur.theta);
                cur.y += ds * std::sin(cur.theta);
                pts.push_back(cur);
            }
        } else {
            double a = sl;
            double arc = a * path.R_min;
            int n = std::max(1, (int)std::ceil(arc / step));
            double da = a / n;
            for (int i = 0; i < n; ++i) {
                if (st == 'L') {
                    cur.x += path.R_min * (std::sin(cur.theta + da) - std::sin(cur.theta));
                    cur.y += path.R_min * (-std::cos(cur.theta + da) + std::cos(cur.theta));
                    cur.theta = mod2pi(cur.theta + da);
                } else {
                    cur.x += path.R_min * (-std::sin(cur.theta - da) + std::sin(cur.theta));
                    cur.y += path.R_min * (std::cos(cur.theta - da) - std::cos(cur.theta));
                    cur.theta = mod2pi(cur.theta - da);
                }
                pts.push_back(cur);
            }
        }
    }
    return pts;
}

int main() {
    Pose2 q0{0.0, 0.0, 10.0 * PI / 180.0};
    Pose2 q1{8.0, 4.0, 110.0 * PI / 180.0};
    double R_min = 2.0;

    DubinsPath path = dubinsShortest(q0, q1, R_min);
    std::cout << "Dubins type: " << path.type << ", length = " << path.length() << "\n";

    auto pts = samplePath(q0, path, 0.05);

    std::ofstream f("dubins_path.csv");
    f << "x,y,theta\n";
    for (const auto& p : pts) {
        f << p.x << "," << p.y << "," << p.theta << "\n";
    }
    f.close();

    std::cout << "Wrote dubins_path.csv (" << pts.size() << " samples)\n";

    // Optional (exact Reeds–Shepp) via OMPL:
    //   #include <ompl/base/spaces/ReedsSheppStateSpace.h>
    //   Use ompl::base::ReedsSheppStateSpace(R_min) to compute RS distance and interpolate.
    return 0;
}

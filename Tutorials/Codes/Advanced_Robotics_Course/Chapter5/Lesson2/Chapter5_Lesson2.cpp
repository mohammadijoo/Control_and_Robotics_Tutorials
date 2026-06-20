#include <cmath>
#include <vector>
#include <limits>

struct LatticeState {
    int i, j, k; // indices in x, y, theta
};

struct ContinuousState {
    double x, y, th;
};

static const double DX = 1.0;
static const double DY = 1.0;
static const int    N_TH = 16;
static const double DTH = 2.0 * M_PI / N_TH;

double wrapTheta(double th) {
    double twoPi = 2.0 * M_PI;
    th = fmod(th, twoPi);
    if (th < 0.0) th += twoPi;
    return th;
}

LatticeState quantize(const ContinuousState& s) {
    int i = static_cast<int>(std::round(s.x / DX));
    int j = static_cast<int>(std::round(s.y / DY));
    int k = static_cast<int>(std::round(wrapTheta(s.th) / DTH)) % N_TH;
    return {i, j, k};
}

ContinuousState dequantize(const LatticeState& q) {
    ContinuousState s;
    s.x  = q.i * DX;
    s.y  = q.j * DY;
    s.th = q.k * DTH;
    return s;
}

struct Primitive {
    double v;
    double omega;
    double T;
    double dt;
    // displacement from canonical (0,0,0)
    ContinuousState disp;
    double cost;

    Primitive(double v_, double o_, double T_, double dt_ = 0.1)
        : v(v_), omega(o_), T(T_), dt(dt_) {
        simulateCanonical();
    }

    void simulateCanonical() {
        double x = 0.0, y = 0.0, th = 0.0;
        double t = 0.0;
        double c = 0.0;
        while (t < T) {
            x += v * std::cos(th) * dt;
            y += v * std::sin(th) * dt;
            th += omega * dt;
            t  += dt;
            c  += dt;
        }
        th = wrapTheta(th);
        disp = {x, y, th};
        cost = c;
    }
};

std::vector<Primitive> makePrimitiveSet() {
    std::vector<Primitive> prims;
    prims.emplace_back(1.0,  0.0, 1.0);  // straight
    prims.emplace_back(1.0,  0.8, 1.0);  // left
    prims.emplace_back(1.0, -0.8, 1.0);  // right
    return prims;
}

LatticeState applyPrimitive(const LatticeState& q,
                            const Primitive& p) {
    ContinuousState s0 = dequantize(q);
    double th0 = s0.th;

    // rotate primitive displacement into world frame
    double dx = std::cos(th0) * p.disp.x - std::sin(th0) * p.disp.y;
    double dy = std::sin(th0) * p.disp.x + std::cos(th0) * p.disp.y;
    double dth = p.disp.th;

    ContinuousState s1;
    s1.x  = s0.x + dx;
    s1.y  = s0.y + dy;
    s1.th = wrapTheta(th0 + dth);
    return quantize(s1);
}

// A* or D* search would then use makePrimitiveSet() and applyPrimitive()
// to generate successors from each lattice node.
      

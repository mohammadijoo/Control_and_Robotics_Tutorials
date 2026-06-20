// Chapter15_Lesson4.cpp
// Event handling and hybrid simulation from scratch (RK4 + bisection)
// Example: Bouncing ball with restitution

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

struct State {
    double h; // height
    double v; // velocity
};

static constexpr double g = 9.81;
static constexpr double e = 0.82;

State flow(const State& s) {
    return {s.v, -g};
}

State add(const State& a, const State& b, double scale = 1.0) {
    return {a.h + scale * b.h, a.v + scale * b.v};
}

State rk4Step(const State& s, double dt) {
    State k1 = flow(s);
    State k2 = flow(add(s, k1, 0.5 * dt));
    State k3 = flow(add(s, k2, 0.5 * dt));
    State k4 = flow(add(s, k3, dt));
    State out;
    out.h = s.h + (dt / 6.0) * (k1.h + 2.0 * k2.h + 2.0 * k3.h + k4.h);
    out.v = s.v + (dt / 6.0) * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    return out;
}

double eventFunction(const State& s) {
    return s.h; // ground event when h = 0
}

State jumpMap(const State& sMinus) {
    return {0.0, -e * sMinus.v};
}

// Refine event time in [tL, tR] assuming sign change in h and monotone descent near event.
void refineEventBisection(double tL, const State& sL,
                         double tR, const State& sR,
                         double tol,
                         double& tEvent,
                         State& sEvent) {
    double a = tL, b = tR;
    State sa = sL, sb = sR;

    for (int iter = 0; iter < 60; ++iter) {
        double m = 0.5 * (a + b);
        double dt = m - a;
        State sm = rk4Step(sa, dt);

        if (std::abs(eventFunction(sm)) < tol || (b - a) < tol) {
            tEvent = m;
            sEvent = sm;
            return;
        }

        if (eventFunction(sa) * eventFunction(sm) <= 0.0) {
            b = m;
            sb = sm;
        } else {
            a = m;
            sa = sm;
        }
    }

    tEvent = 0.5 * (a + b);
    sEvent = rk4Step(sa, tEvent - a);
}

int main() {
    const double tFinal = 8.0;
    const double dt = 0.01;
    const double eventTol = 1e-10;
    const double minBounceSpeed = 0.05;

    double t = 0.0;
    State s{1.5, 0.0};

    std::ofstream csv("Chapter15_Lesson4_cpp_output.csv");
    csv << "t,h,v\n";
    csv << std::setprecision(12);

    csv << t << "," << s.h << "," << s.v << "\n";

    while (t < tFinal) {
        State sNext = rk4Step(s, dt);
        double g0 = eventFunction(s);
        double g1 = eventFunction(sNext);

        // Check for zero crossing while descending
        if (g0 > 0.0 && g1 <= 0.0 && s.v < 0.0) {
            double tEvent;
            State sMinus;
            refineEventBisection(t, s, t + dt, sNext, eventTol, tEvent, sMinus);

            csv << tEvent << "," << sMinus.h << "," << sMinus.v << "\n";

            if (std::abs(sMinus.v) < minBounceSpeed) {
                State rest{0.0, 0.0};
                csv << tEvent << "," << rest.h << "," << rest.v << "\n";
                break;
            }

            State sPlus = jumpMap(sMinus);
            t = tEvent;
            s = sPlus;
            csv << t << "," << s.h << "," << s.v << "\n";
            continue;
        }

        t += dt;
        s = sNext;
        csv << t << "," << s.h << "," << s.v << "\n";
    }

    csv.close();
    std::cout << "Simulation complete. Results written to Chapter15_Lesson4_cpp_output.csv\n";
    return 0;
}

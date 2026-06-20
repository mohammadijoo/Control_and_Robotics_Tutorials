// Chapter12_Lesson2.cpp
// Loop Closure Detection Concepts — minimal C++ demo (descriptor retrieval + chi-square gating)
// Build (example):
//   g++ -O2 -std=c++17 Chapter12_Lesson2.cpp -I /usr/include/eigen3 -o lc_demo
//
// Notes:
// - Uses Eigen for vector/matrix operations.
// - Retrieval is brute-force cosine similarity for clarity.

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

struct Pose2 {
    double x{0}, y{0}, th{0};
};

static double wrap(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

static std::vector<Pose2> makeSquareLoop(int N, double side) {
    std::vector<Pose2> gt(N);
    double perim = 4.0*side;
    for (int i = 0; i < N; ++i) {
        double s = (perim * i) / N;
        int seg = static_cast<int>(std::floor(s / side));
        double u = std::fmod(s, side);
        Pose2 p;
        if (seg == 0)      { p.x = u;        p.y = 0.0;      p.th = 0.0; }
        else if (seg == 1) { p.x = side;     p.y = u;        p.th = M_PI/2; }
        else if (seg == 2) { p.x = side-u;   p.y = side;     p.th = M_PI; }
        else               { p.x = 0.0;      p.y = side-u;   p.th = -M_PI/2; }
        gt[i] = p;
    }
    return gt;
}

static std::vector<Pose2> addOdometryDrift(const std::vector<Pose2>& gt,
                                          double sigma_xy, double sigma_th,
                                          double drift_x, double drift_y, double drift_th) {
    int N = static_cast<int>(gt.size());
    std::vector<Pose2> odo(N);
    odo[0] = gt[0];

    std::mt19937 rng(0);
    std::normal_distribution<double> nxy(0.0, sigma_xy);
    std::normal_distribution<double> nth(0.0, sigma_th);

    for (int k = 1; k < N; ++k) {
        Pose2 d;
        d.x = gt[k].x - gt[k-1].x;
        d.y = gt[k].y - gt[k-1].y;
        d.th = wrap(gt[k].th - gt[k-1].th);

        Pose2 inc;
        inc.x = d.x + nxy(rng) + drift_x;
        inc.y = d.y + nxy(rng) + drift_y;
        inc.th = d.th + nth(rng) + drift_th;

        odo[k].x = odo[k-1].x + inc.x;
        odo[k].y = odo[k-1].y + inc.y;
        odo[k].th = wrap(odo[k-1].th + inc.th);
    }
    return odo;
}

static double cosineSim(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    double na = a.norm(), nb = b.norm();
    if (na < 1e-12 || nb < 1e-12) return 0.0;
    return (a.dot(b)) / (na*nb);
}

static bool chiSquareGate(const Eigen::Vector3d& r,
                          double sigma_xy, double sigma_th,
                          double thr) {
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S(0,0) = sigma_xy*sigma_xy;
    S(1,1) = sigma_xy*sigma_xy;
    S(2,2) = sigma_th*sigma_th;
    double d2 = r.transpose() * S.inverse() * r;
    return d2 < thr;
}

// Hard-coded chi-square 0.995 quantile for df=3 (approx 12.838)
// (Avoids bringing a statistics library into the example.)
static constexpr double CHI2_3_0995 = 12.838;

int main() {
    const int N = 420;
    const int V = 300;
    const int MIN_SEP = 40;
    const double SIDE = 25.0;

    auto gt = makeSquareLoop(N, SIDE);
    auto odo = addOdometryDrift(gt, 0.03, 0.003, 0.002, -0.001, 0.0002);

    // Toy descriptors: one vector per pose (sparse-ish) with place prototypes.
    std::mt19937 rng(1);
    std::uniform_int_distribution<int> uid(0, V-1);
    std::uniform_real_distribution<double> ur(0.5, 2.0);
    std::poisson_distribution<int> pois(1);

    const int N_PLACES = 90;
    std::vector<Eigen::VectorXd> prot(N_PLACES, Eigen::VectorXd::Zero(V));
    for (int p = 0; p < N_PLACES; ++p) {
        for (int t = 0; t < 20; ++t) {
            int idx = uid(rng);
            prot[p](idx) = ur(rng);
        }
        prot[p].normalize();
    }

    auto placeId = [&](double x, double y) {
        int qx = static_cast<int>(std::floor(x/3.2));
        int qy = static_cast<int>(std::floor(y/3.2));
        long long h = 1LL*qx*73856093LL + 1LL*qy*19349663LL;
        int pid = static_cast<int>((h % N_PLACES + N_PLACES) % N_PLACES);
        return pid;
    };

    std::vector<Eigen::VectorXd> X(N, Eigen::VectorXd::Zero(V));
    for (int i = 0; i < N; ++i) {
        int pid = placeId(gt[i].x, gt[i].y);
        Eigen::VectorXd c = Eigen::VectorXd::Zero(V);
        for (int v = 0; v < V; ++v) {
            double lam = 8.0*prot[pid](v) + 0.15;
            std::poisson_distribution<int> ppois(lam);
            c(v) = static_cast<double>(ppois(rng));
        }
        // simple TF normalization (skip IDF to keep code compact)
        double s = c.sum();
        if (s > 1e-12) c /= s;
        X[i] = c;
    }

    // Ground-truth loop edges
    std::vector<std::pair<int,int>> gtLoops;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < i - MIN_SEP; ++j) {
            double dx = gt[i].x - gt[j].x;
            double dy = gt[i].y - gt[j].y;
            if (std::sqrt(dx*dx + dy*dy) < 1.4) gtLoops.push_back({i,j});
        }
    }

    // Retrieval + verification
    int detected = 0, tp = 0, fp = 0;
    for (int i = 0; i < N; ++i) {
        int jmax = i - MIN_SEP;
        if (jmax <= 0) continue;

        // best match by cosine
        double bestSim = -1.0;
        int bestJ = -1;
        for (int j = 0; j < jmax; ++j) {
            double sim = cosineSim(X[i], X[j]);
            if (sim > bestSim) { bestSim = sim; bestJ = j; }
        }

        // similarity threshold (tune)
        if (bestSim < 0.75) continue;

        // geometric gating (use gt-based measurement around true rel pose)
        std::normal_distribution<double> nxy(0.0, 0.20);
        std::normal_distribution<double> nth(0.0, 0.07);

        Eigen::Vector3d pred;
        pred << (odo[i].x - odo[bestJ].x), (odo[i].y - odo[bestJ].y), wrap(odo[i].th - odo[bestJ].th);

        Eigen::Vector3d z;
        z << (gt[i].x - gt[bestJ].x) + nxy(rng),
             (gt[i].y - gt[bestJ].y) + nxy(rng),
             wrap((gt[i].th - gt[bestJ].th) + nth(rng));

        Eigen::Vector3d r = z - pred;
        r(2) = wrap(r(2));

        if (!chiSquareGate(r, 0.20, 0.07, CHI2_3_0995)) continue;

        detected++;

        // Determine TP/FP
        bool isTrue = false;
        for (auto& e : gtLoops) {
            if (e.first == i && e.second == bestJ) { isTrue = true; break; }
        }
        if (isTrue) tp++; else fp++;
    }

    double prec = tp / (double)(tp + fp + 1e-12);
    double rec  = tp / (double)(gtLoops.size() + 1e-12);

    std::cout << "N=" << N << "\n";
    std::cout << "GT loops: " << gtLoops.size() << "\n";
    std::cout << "Detected: " << detected << "\n";
    std::cout << "Precision: " << prec << "  Recall: " << rec << "\n";
    return 0;
}

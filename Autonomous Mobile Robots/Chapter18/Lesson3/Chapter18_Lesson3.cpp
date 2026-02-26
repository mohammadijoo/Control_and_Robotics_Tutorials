// Chapter18_Lesson3.cpp
// Long-Range Localization Drift Handling (2D EKF, no external dependencies)
// State: [x, y, psi, b_g]^T
// Compile: g++ -O2 -std=c++17 Chapter18_Lesson3.cpp -o Chapter18_Lesson3

#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <numeric>

using Vec4 = std::array<double, 4>;
using Mat4 = std::array<std::array<double, 4>, 4>;

static constexpr double PI = 3.14159265358979323846;

double wrapAngle(double a) {
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

Mat4 eye4() {
    Mat4 I{};
    for (int i = 0; i < 4; ++i) I[i][i] = 1.0;
    return I;
}

Mat4 matAdd(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Mat4 matSub(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            C[i][j] = A[i][j] - B[i][j];
    return C;
}

Mat4 matMul(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 4; ++k)
            for (int j = 0; j < 4; ++j)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Mat4 matT(const Mat4& A) {
    Mat4 T{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = A[j][i];
    return T;
}

void symmetrize(Mat4& P) {
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            double s = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = s;
            P[j][i] = s;
        }
    }
}

double trace4(const Mat4& A) {
    return A[0][0] + A[1][1] + A[2][2] + A[3][3];
}

struct FilterState {
    Vec4 x;
    Mat4 P;
};

void propagate(FilterState& s, double vm, double wm, double dt, bool rough) {
    double psi = s.x[2];
    double bg  = s.x[3];

    // Nominal propagation
    s.x[0] += vm * dt * std::cos(psi);
    s.x[1] += vm * dt * std::sin(psi);
    s.x[2] = wrapAngle(s.x[2] + (wm - bg) * dt);

    // Jacobian
    Mat4 F = eye4();
    F[0][2] = -vm * dt * std::sin(psi);
    F[1][2] =  vm * dt * std::cos(psi);
    F[2][3] = -dt;

    // Process noise
    double sigmaV = rough ? 0.12 : 0.03;
    double sigmaW = rough ? 0.06 : 0.01;
    double sigmaBg = rough ? 0.0030 : 0.0008;

    Mat4 Q{};
    Q[0][0] = std::pow(sigmaV * dt, 2);
    Q[1][1] = std::pow(sigmaV * dt, 2);
    Q[2][2] = std::pow(sigmaW * dt, 2);
    Q[3][3] = std::pow(sigmaBg, 2) * dt;

    s.P = matAdd(matMul(matMul(F, s.P), matT(F)), Q);
    symmetrize(s.P);
}

double scalarUpdate(FilterState& s, int idx, double z, double R, bool angleResidual) {
    double innov = z - s.x[idx];
    if (angleResidual) innov = wrapAngle(innov);

    double S = s.P[idx][idx] + R;

    // K = P[:,idx] / S
    double K[4];
    for (int i = 0; i < 4; ++i) K[i] = s.P[i][idx] / S;

    // x = x + K * innov
    for (int i = 0; i < 4; ++i) s.x[i] += K[i] * innov;
    s.x[2] = wrapAngle(s.x[2]);

    // P = P - K * P[idx,:]
    Mat4 Pnew = s.P;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Pnew[i][j] -= K[i] * s.P[idx][j];
        }
    }
    s.P = Pnew;
    symmetrize(s.P);

    return (innov * innov) / S; // scalar NIS
}

int main() {
    const double dt = 0.1;
    const int N = 1800;
    const double bgTrue = 0.015;

    std::mt19937 rng(18);
    std::normal_distribution<double> n01(0.0, 1.0);

    std::vector<Vec4> xTrue(N, Vec4{0.0, 0.0, 0.0, bgTrue});
    std::vector<Vec4> xEstHist(N, Vec4{0.0, 0.0, 0.0, 0.0});
    std::vector<double> Ptrace(N, 0.0);
    std::vector<double> nisGnss, nisAnchor;

    FilterState f;
    f.x = {0.0, 0.0, 0.05, 0.0};
    f.P = {};
    f.P[0][0] = 1.0;
    f.P[1][1] = 1.0;
    f.P[2][2] = std::pow(5.0 * PI / 180.0, 2);
    f.P[3][3] = std::pow(0.03, 2);

    auto isRough = [](int k) {
        return (300 <= k && k < 520) || (840 <= k && k < 1100) || (1400 <= k && k < 1620);
    };

    auto inOutage = [](int k) {
        bool o1 = (420 <= k && k < 980);
        bool o2 = (1250 <= k && k < 1500);
        return o1 || o2;
    };

    for (int k = 1; k < N; ++k) {
        double tk = k * dt;
        double vCmd = 1.5 + 0.3 * std::sin(0.015 * tk);
        double wCmd = 0.08 * std::sin(0.010 * tk) + 0.04 * std::sin(0.040 * tk);

        bool rough = isRough(k);

        double sigmaWTrue = rough ? 0.03 : 0.008;
        double sigmaVTrue = rough ? 0.10 : 0.02;

        double vTrue = vCmd + sigmaVTrue * n01(rng);
        double wTrue = wCmd + sigmaWTrue * n01(rng);

        xTrue[k][0] = xTrue[k-1][0] + vTrue * dt * std::cos(xTrue[k-1][2]);
        xTrue[k][1] = xTrue[k-1][1] + vTrue * dt * std::sin(xTrue[k-1][2]);
        xTrue[k][2] = wrapAngle(xTrue[k-1][2] + (wTrue - bgTrue) * dt);
        xTrue[k][3] = bgTrue;

        double vm = vTrue + (rough ? 0.12 : 0.03) * n01(rng);
        double wm = wTrue + (rough ? 0.05 : 0.01) * n01(rng);

        propagate(f, vm, wm, dt, rough);

        // GNSS/RTK sequential scalar updates (x, y)
        if ((k % 10 == 0) && !inOutage(k)) {
            bool rtkFixed = (!rough) && (k % 90 != 0);
            double sigmaGps = rtkFixed ? 0.15 : 0.75;
            double zgx = xTrue[k][0] + sigmaGps * n01(rng);
            double zgy = xTrue[k][1] + sigmaGps * n01(rng);

            double innovX = zgx - f.x[0];
            double Sx = f.P[0][0] + sigmaGps * sigmaGps;
            double d2x = (innovX * innovX) / Sx;
            if (d2x < 6.63) {
                nisGnss.push_back(scalarUpdate(f, 0, zgx, sigmaGps * sigmaGps, false));
            }

            double innovY = zgy - f.x[1];
            double Sy = f.P[1][1] + sigmaGps * sigmaGps;
            double d2y = (innovY * innovY) / Sy;
            if (d2y < 6.63) {
                nisGnss.push_back(scalarUpdate(f, 1, zgy, sigmaGps * sigmaGps, false));
            }
        }

        // Anchor re-localization at selected times (x, y, psi)
        if (k == 1000 || k == 1510 || k == 1710) {
            double zax = xTrue[k][0] + 0.10 * n01(rng);
            double zay = xTrue[k][1] + 0.10 * n01(rng);
            double zap = wrapAngle(xTrue[k][2] + (2.0 * PI / 180.0) * n01(rng));

            nisAnchor.push_back(scalarUpdate(f, 0, zax, 0.10 * 0.10, false));
            nisAnchor.push_back(scalarUpdate(f, 1, zay, 0.10 * 0.10, false));
            nisAnchor.push_back(scalarUpdate(f, 2, zap, std::pow(2.0 * PI / 180.0, 2), true));
        }

        // Integrity-based covariance inflation
        if (nisGnss.size() >= 20) {
            double sum = 0.0;
            for (size_t i = nisGnss.size() - 20; i < nisGnss.size(); ++i) sum += nisGnss[i];
            double meanNIS = sum / 20.0;
            if (meanNIS > 1.8) {
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        f.P[i][j] *= 1.02;
            }
        }

        xEstHist[k] = f.x;
        Ptrace[k] = trace4(f.P);
    }

    // Metrics
    double sumSq = 0.0;
    for (int k = 0; k < N; ++k) {
        double ex = xEstHist[k][0] - xTrue[k][0];
        double ey = xEstHist[k][1] - xTrue[k][1];
        sumSq += ex * ex + ey * ey;
    }
    double rmse = std::sqrt(sumSq / N);

    double exf = xEstHist[N-1][0] - xTrue[N-1][0];
    double eyf = xEstHist[N-1][1] - xTrue[N-1][1];
    double finalErr = std::sqrt(exf * exf + eyf * eyf);
    double finalHeadingDeg = wrapAngle(xEstHist[N-1][2] - xTrue[N-1][2]) * 180.0 / PI;

    auto meanVec = [](const std::vector<double>& v) {
        if (v.empty()) return std::nan("");
        double s = std::accumulate(v.begin(), v.end(), 0.0);
        return s / static_cast<double>(v.size());
    };

    std::cout << "=== Chapter18_Lesson3.cpp : Long-Range Localization Drift Handling ===\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Mission duration: " << N * dt << " s\n";
    std::cout << "Position RMSE: " << rmse << " m\n";
    std::cout << "Final position error: " << finalErr << " m\n";
    std::cout << "Final heading error: " << finalHeadingDeg << " deg\n";
    std::cout << "Mean scalar NIS (GNSS accepted): " << meanVec(nisGnss) << "\n";
    std::cout << "Mean scalar NIS (Anchor): " << meanVec(nisAnchor) << "\n";
    std::cout << "Final covariance trace: " << Ptrace[N-1] << "\n";

    auto outageReport = [&](const char* name, int a, int b) {
        double esx = xEstHist[a][0] - xTrue[a][0];
        double esy = xEstHist[a][1] - xTrue[a][1];
        double eex = xEstHist[b-1][0] - xTrue[b-1][0];
        double eey = xEstHist[b-1][1] - xTrue[b-1][1];
        double estart = std::sqrt(esx * esx + esy * esy);
        double eend = std::sqrt(eex * eex + eey * eey);
        std::cout << name << ": error at start=" << estart
                  << " m, end=" << eend
                  << " m, growth=" << (eend - estart) << " m\n";
    };
    outageReport("Outage1", 420, 980);
    outageReport("Outage2", 1250, 1500);

    return 0;
}

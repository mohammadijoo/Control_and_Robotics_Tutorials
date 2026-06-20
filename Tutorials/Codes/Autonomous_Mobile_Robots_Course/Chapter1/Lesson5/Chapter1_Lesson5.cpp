// Chapter1_Lesson5.cpp
/*
Autonomous Mobile Robots — Chapter 1, Lesson 5
Typical AMR Failure Modes (drift, slip, occlusion)

This C++ program mirrors the Python simulation at a lightweight level:
  - Simulates true unicycle motion with slip (attenuation + bursts)
  - Simulates biased wheel/gyro odometry (drift)
  - Generates intermittent landmark range/bearing measurements with occlusion/outliers
  - Applies a single-step linearized correction with gating

Build (example):
  g++ -O2 -std=c++17 Chapter1_Lesson5.cpp -o Chapter1_Lesson5

Outputs:
  trajectory.csv with columns:
    t, x_true, y_true, th_true, x_hat, y_hat, th_hat, meas_r, meas_b, meas_used
*/

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <array>

static double wrap_to_pi(double a) {
    const double two_pi = 2.0 * M_PI;
    a = std::fmod(a + M_PI, two_pi);
    if (a < 0) a += two_pi;
    return a - M_PI;
}

static std::array<double,3> unicycle_step(const std::array<double,3>& x,
                                          const std::array<double,2>& u,
                                          double dt) {
    const double px = x[0], py = x[1], th = x[2];
    const double v  = u[0], w  = u[1];
    return { px + dt * v * std::cos(th),
             py + dt * v * std::sin(th),
             wrap_to_pi(th + dt * w) };
}

static std::array<double,2> landmark_meas(const std::array<double,3>& x,
                                          const std::array<double,2>& lm) {
    const double dx = lm[0] - x[0];
    const double dy = lm[1] - x[1];
    const double r  = std::sqrt(dx*dx + dy*dy);
    const double b  = wrap_to_pi(std::atan2(dy, dx) - x[2]);
    return {r, b};
}

// 2x3 Jacobian for range/bearing
static void jacobian_landmark(const std::array<double,3>& x,
                              const std::array<double,2>& lm,
                              double H[2][3]) {
    const double px=x[0], py=x[1];
    const double dx = lm[0] - px;
    const double dy = lm[1] - py;
    double q = dx*dx + dy*dy;
    if (q < 1e-12) q = 1e-12;
    double r = std::sqrt(q);
    if (r < 1e-12) r = 1e-12;

    // dr/dx, dr/dy
    H[0][0] = -dx / r;
    H[0][1] = -dy / r;
    H[0][2] =  0.0;

    // db/dx, db/dy, db/dth
    H[1][0] =  dy / q;
    H[1][1] = -dx / q;
    H[1][2] = -1.0;
}

// small 3x3 matrix helpers (row-major)
static void mat33_add_diag(double P[3][3], double d0, double d1, double d2) {
    P[0][0] += d0; P[1][1] += d1; P[2][2] += d2;
}

static void mat33_identity(double I[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) I[i][j]=(i==j)?1.0:0.0;
}

// Compute S = H P H^T + R, where H is 2x3, P is 3x3, R is 2x2 diagonal
static void compute_S(const double H[2][3], const double P[3][3],
                      double R00, double R11, double S[2][2]) {
    // HP = H*P (2x3)
    double HP[2][3] = {{0,0,0},{0,0,0}};
    for (int i=0;i<2;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                HP[i][j] += H[i][k]*P[k][j];
            }
        }
    }
    // S = HP*H^T + R (2x2)
    for (int i=0;i<2;i++){
        for (int j=0;j<2;j++){
            double s=0.0;
            for (int k=0;k<3;k++){
                s += HP[i][k]*H[j][k];
            }
            S[i][j]=s;
        }
    }
    S[0][0] += R00;
    S[1][1] += R11;
}

// Invert 2x2
static bool inv22(const double A[2][2], double Ainv[2][2]) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (std::fabs(det) < 1e-12) return false;
    double invdet = 1.0/det;
    Ainv[0][0] =  A[1][1]*invdet;
    Ainv[0][1] = -A[0][1]*invdet;
    Ainv[1][0] = -A[1][0]*invdet;
    Ainv[1][1] =  A[0][0]*invdet;
    return true;
}

// K = P H^T S^{-1}  (3x2)
static void compute_K(const double P[3][3], const double H[2][3],
                      const double Sinv[2][2], double K[3][2]) {
    // PH^T (3x2)
    double PHT[3][2] = {{0,0},{0,0},{0,0}};
    for (int i=0;i<3;i++){
        for (int j=0;j<2;j++){
            for (int k=0;k<3;k++){
                PHT[i][j] += P[i][k]*H[j][k]; // H^T has entry H[j][k]
            }
        }
    }
    // K = PHT * Sinv
    for (int i=0;i<3;i++){
        for (int j=0;j<2;j++){
            K[i][j]=0.0;
            for (int k=0;k<2;k++){
                K[i][j] += PHT[i][k]*Sinv[k][j];
            }
        }
    }
}

int main() {
    const double T = 40.0;
    const double dt = 0.02;
    const int N = static_cast<int>(T/dt);

    std::mt19937 gen(7);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    std::array<double,2> landmark{8.0, 6.0};

    // states
    std::array<double,3> x_true{0.0, 0.0, 0.0};
    std::array<double,3> x_hat {0.0, 0.0, 0.0};

    // drift parameters
    const double b_v = 0.03, b_w = -0.015;
    const double s_v = 1.03, s_w = 0.98;

    // slip
    const double s_long_nom=0.05, s_yaw_nom=0.03;
    const double slip_burst_prob=0.02, slip_burst_mag=0.35;

    // noise
    const double odo_sigma_v=0.02, odo_sigma_w=0.02;
    const double meas_sigma_r=0.08;
    const double meas_sigma_b = (1.5*M_PI/180.0);

    // correction
    const double R00 = meas_sigma_r*meas_sigma_r;
    const double R11 = meas_sigma_b*meas_sigma_b;
    double P[3][3] = {{0.04,0,0},{0,0.04,0},{0,0,(5.0*M_PI/180.0)*(5.0*M_PI/180.0)}};
    const double gate = 9.21;

    // measurement schedule
    const double meas_period=0.5;
    double next_meas_t=0.0;
    const double p_occlude=0.35;
    const double p_outlier=0.08;

    std::ofstream f("trajectory.csv");
    f << "t,x_true,y_true,th_true,x_hat,y_hat,th_hat,meas_r,meas_b,meas_used\n";

    auto u_cmd = [](double t) -> std::array<double,2> {
        double v = 0.6 + 0.15*std::sin(0.4*t);
        double w = 0.25*std::sin(0.25*t) + 0.20*std::sin(0.05*t);
        return {v,w};
    };

    for (int k=0;k<N;k++){
        double t = k*dt;
        auto u = u_cmd(t);

        // slip
        bool burst = (unif(gen) < slip_burst_prob);
        double s_long = s_long_nom + (burst ? slip_burst_mag : 0.0);
        double s_yaw  = s_yaw_nom  + (burst ? 0.5*slip_burst_mag : 0.0);
        if (s_long > 0.9) s_long = 0.9;
        if (s_yaw  > 0.9) s_yaw  = 0.9;

        std::array<double,2> u_true{ (1.0 - s_long)*u[0], (1.0 - s_yaw)*u[1] };
        x_true = unicycle_step(x_true, u_true, dt);

        // odometry (biased)
        std::array<double,2> u_odo{
            s_v*u[0] + b_v + odo_sigma_v*n01(gen),
            s_w*u[1] + b_w + odo_sigma_w*n01(gen)
        };
        x_hat = unicycle_step(x_hat, u_odo, dt);

        double meas_r = NAN, meas_b = NAN;
        int meas_used = 0;

        if (t + 1e-12 >= next_meas_t){
            next_meas_t += meas_period;
            bool occluded = (unif(gen) < p_occlude);
            if (!occluded){
                auto z = landmark_meas(x_true, landmark);
                // noise
                z[0] += meas_sigma_r*n01(gen);
                z[1] = wrap_to_pi(z[1] + meas_sigma_b*n01(gen));
                // outlier
                if (unif(gen) < p_outlier){
                    z[0] += 2.0*n01(gen);
                    z[1] = wrap_to_pi(z[1] + (25.0*M_PI/180.0)*n01(gen));
                }
                meas_r = z[0]; meas_b = z[1];

                auto zhat = landmark_meas(x_hat, landmark);
                double r0 = z[0] - zhat[0];
                double r1 = wrap_to_pi(z[1] - zhat[1]);

                double H[2][3];
                jacobian_landmark(x_hat, landmark, H);

                double S[2][2];
                compute_S(H, P, R00, R11, S);
                double Sinv[2][2];
                if (inv22(S, Sinv)){
                    // d^2 = r^T S^{-1} r
                    double d2 = r0*(Sinv[0][0]*r0 + Sinv[0][1]*r1) +
                                r1*(Sinv[1][0]*r0 + Sinv[1][1]*r1);

                    if (d2 < gate){
                        double K[3][2];
                        compute_K(P, H, Sinv, K);

                        // dx = K r
                        double dx0 = K[0][0]*r0 + K[0][1]*r1;
                        double dx1 = K[1][0]*r0 + K[1][1]*r1;
                        double dx2 = K[2][0]*r0 + K[2][1]*r1;

                        x_hat[0] += dx0;
                        x_hat[1] += dx1;
                        x_hat[2]  = wrap_to_pi(x_hat[2] + dx2);

                        meas_used = 1;

                        // crude covariance update: P <- (I-KH)P(I-KH)^T + K R K^T
                        double I[3][3]; mat33_identity(I);
                        double KH[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                for (int m=0;m<2;m++){
                                    KH[i][j] += K[i][m]*H[m][j];
                                }
                            }
                        }
                        double A[3][3];
                        for (int i=0;i<3;i++) for (int j=0;j<3;j++) A[i][j] = I[i][j] - KH[i][j];

                        // temp = A*P
                        double temp[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                for (int m=0;m<3;m++){
                                    temp[i][j] += A[i][m]*P[m][j];
                                }
                            }
                        }
                        // newP = temp*A^T
                        double newP[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                for (int m=0;m<3;m++){
                                    newP[i][j] += temp[i][m]*A[j][m]; // A^T
                                }
                            }
                        }
                        // add K R K^T (R diagonal)
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                newP[i][j] += K[i][0]*R00*K[j][0] + K[i][1]*R11*K[j][1];
                            }
                        }
                        // copy
                        for (int i=0;i<3;i++) for (int j=0;j<3;j++) P[i][j]=newP[i][j];
                    }
                }
            }
        }

        // mild inflation
        mat33_add_diag(P, 1e-4, 1e-4, 1e-6);

        f << t << ","
          << x_true[0] << "," << x_true[1] << "," << x_true[2] << ","
          << x_hat[0]  << "," << x_hat[1]  << "," << x_hat[2]  << ","
          << meas_r << "," << meas_b << "," << meas_used << "\n";
    }

    f.close();
    std::cout << "Wrote trajectory.csv\n";
    return 0;
}

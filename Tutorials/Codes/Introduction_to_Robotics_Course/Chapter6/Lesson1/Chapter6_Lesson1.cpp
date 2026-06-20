#include <iostream>
#include <vector>
#include <cmath>

struct DCMotor {
    double R, L, ke, kt, J, b;
    double i = 0.0;
    double omega = 0.0;

    void step(double v, double dt) {
        double di = (v - R*i - ke*omega)/L;
        i += di*dt;
        double domega = (kt*i - b*omega)/J;
        omega += domega*dt;
    }
};

struct PID {
    double Kp, Ki, Kd;
    double eint = 0.0, eprev = 0.0;

    double update(double e, double dt) {
        eint += e*dt;
        double eder = (e - eprev)/dt;
        eprev = e;
        return Kp*e + Ki*eint + Kd*eder;
    }
};

int main() {
    DCMotor m{1.2, 2e-3, 0.08, 0.08, 5e-4, 1e-3};
    PID pid{0.4, 30.0, 0.0};

    double dt = 1e-4, Tend = 0.5;
    int N = (int)(Tend/dt);
    double omega_ref = 100.0;

    for(int k=0; k<N; ++k){
        double e = omega_ref - m.omega;
        double v = pid.update(e, dt);
        m.step(v, dt);
        if(k % 1000 == 0){
            std::cout << k*dt << "," << m.omega << std::endl;
        }
    }
    return 0;
}

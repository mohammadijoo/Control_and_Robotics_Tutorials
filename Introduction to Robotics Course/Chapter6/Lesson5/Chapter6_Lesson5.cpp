#include <iostream>
#include <array>

struct SEA {
    double Jm, Jl, bm, bl, N, ks;
    std::array<double,4> x; // [theta_m, omega_m, theta_l, omega_l]

    SEA(double Jm_, double Jl_, double bm_, double bl_, double N_, double ks_)
        : Jm(Jm_), Jl(Jl_), bm(bm_), bl(bl_), N(N_), ks(ks_), x{0,0,0,0} {}

    double springTorque() const {
        double theta_s = N*x[0] - x[2];
        return ks*theta_s;
    }

    void step(double tau_m, double dt) {
        double tau_s = springTorque();
        double domega_m = (tau_m - bm*x[1] - N*tau_s) / Jm;
        double domega_l = (tau_s - bl*x[3]) / Jl;

        x[1] += dt*domega_m;
        x[0] += dt*x[1];
        x[3] += dt*domega_l;
        x[2] += dt*x[3];
    }
};

int main(){
    SEA sea(0.01,0.05,0.02,0.05,50.0,200.0);

    double Kp=5.0, Ki=40.0, Kd=0.02;
    double dt=1e-3, T=2.0;
    int steps = (int)(T/dt);
    double eint=0, eprev=0;

    for(int k=0;k<steps;k++){
        double t=k*dt;
        double tau_d = (t>0.2)?2.0:0.0;
        double tau_s = sea.springTorque();
        double e = tau_d - tau_s;
        eint += e*dt;
        double de = (e-eprev)/dt; eprev=e;
        double tau_m = Kp*e + Ki*eint + Kd*de;
        sea.step(tau_m, dt);
    }
    std::cout << "Final tau_s=" << sea.springTorque() << std::endl;
}

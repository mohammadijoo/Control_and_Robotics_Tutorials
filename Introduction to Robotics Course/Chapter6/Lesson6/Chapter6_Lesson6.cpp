#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

struct Motor {
    string name;
    double tau_cont, tau_peak, omega_max, Jm, mass, cost, eta;
};

int main() {
    double tauL_peak=35.0, omegaL_peak=4.0, alphaL_peak=12.0;
    double eta_g=0.92;
    vector<int> gears = {5,8,12,16};

    vector<Motor> catalog = {
        {"BLDC-A", 12, 40, 80, 0.0008, 1.2, 300, 0.90},
        {"Servo-B",20, 60, 40, 0.0016, 2.2, 550, 0.85},
        {"Hydro-C",50,120, 25, 0.0030, 6.0, 900, 0.70}
    };

    double tau_samples[] = {0,10,30,10,0};
    double mean_sq=0;
    for(double t: tau_samples) mean_sq += t*t;
    double tau_rms_req = sqrt(mean_sq/5.0);

    for(const auto& m: catalog){
        for(int g: gears){
            double omega_m_peak = g * omegaL_peak;
            double tau_m_peak = (tauL_peak/(g*eta_g)) + g*m.Jm*alphaL_peak;
            if(tau_m_peak <= m.tau_peak &&
               omega_m_peak <= m.omega_max &&
               tau_rms_req <= m.tau_cont){
                cout << "Feasible: " << m.name << " gear " << g << endl;
            }
        }
    }
    return 0;
}
      

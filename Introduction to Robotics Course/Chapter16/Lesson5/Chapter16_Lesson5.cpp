#include <iostream>
#include <vector>

double availability(double mtbf, double mttr) {
    return mtbf / (mtbf + mttr);
}

double system_availability_series(const std::vector<double>& mtbf,
                                  const std::vector<double>& mttr) {
    if (mtbf.size() != mttr.size()) {
        throw std::runtime_error("mtbf and mttr vectors must have same size");
    }
    double A_sys = 1.0;
    for (std::size_t i = 0; i < mtbf.size(); ++i) {
        double Ai = availability(mtbf[i], mttr[i]);
        A_sys *= Ai;
    }
    return A_sys;
}

int main() {
    std::vector<double> mtbf = {2000.0, 5000.0, 3000.0}; // hours
    std::vector<double> mttr = { 4.0,    2.0,    1.0};   // hours

    double A_sys = system_availability_series(mtbf, mttr);
    std::cout << "Approx series system availability: "
              << A_sys << std::endl;

    double downtime_cost_per_hour = 100.0; // arbitrary units
    double c_down = downtime_cost_per_hour * (1.0 - A_sys);
    std::cout << "Approx downtime cost rate: "
              << c_down << " cost units per hour" << std::endl;
    return 0;
}
      

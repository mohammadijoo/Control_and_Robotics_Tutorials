
#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>

struct ArmStructure {
    std::string seq; // "PPP", "RPP", "RRP", "RRP_SCARA"
    std::unordered_map<std::string,double> p;

    int dof() const { return (int)seq.size(); }

    double workspaceVolume() const {
        const double PI = 3.141592653589793;
        if (seq == "PPP") {
            return p.at("Lx") * p.at("Ly") * p.at("Lz");
        }
        if (seq == "RPP") {
            double rmin = p.at("rmin"), rmax = p.at("rmax"), Lz = p.at("Lz");
            return PI * (rmax*rmax - rmin*rmin) * Lz;
        }
        if (seq == "RRP") {
            double rmin = p.at("rmin"), rmax = p.at("rmax");
            double phimin = p.at("phimin"), phimax = p.at("phimax");
            return 2*PI*(std::cos(phimin) - std::cos(phimax))*(std::pow(rmax,3)-std::pow(rmin,3))/3.0;
        }
        if (seq == "RRP_SCARA") {
            double l1 = p.at("l1"), l2 = p.at("l2"), Lz = p.at("Lz");
            double A = PI * ((l1+l2)*(l1+l2) - std::abs(l1-l2)*std::abs(l1-l2));
            return A * Lz;
        }
        return NAN;
    }
};

int main() {
    ArmStructure gantry{"PPP", {
      {"Lx",1.5},
      {"Ly",1.0},
      {"Lz",0.8}
    }
  };
    std::cout << "Gantry DOF=" << gantry.dof()
              << ", V=" << gantry.workspaceVolume() << std::endl;
    return 0;
}
      
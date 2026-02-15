
#include <iostream>
#include <vector>
#include <string>

struct Module {
    std::string name;
    double stiffness;   // interface stiffness to next module (N/m)
    double powerDraw;   // W
    int mechType;       // simple mechanical connector type id
};

bool compatible(const Module& a, const Module& b) {
    return a.mechType == b.mechType; // toy rule: same connector type
}

double seriesStiffness(const std::vector<Module>& chain) {
    double invSum = 0.0;
    for (size_t i = 0; i < chain.size(); ++i)
        invSum += 1.0 / chain[i].stiffness;
    return 1.0 / invSum;
}

int main() {
    Module m1{"LinkA", 3000, 10, 1};
    Module m2{"JointA", 2000, 25, 1};
    Module m3{"LinkB", 1500, 10, 1};

    if (!compatible(m1, m2) || !compatible(m2, m3)) {
        std::cout << "Incompatible modules!\\n";
        return 0;
    }

    std::vector<Module> chain{m1, m2, m3};
    std::cout << "Equivalent stiffness: " <<
        seriesStiffness(chain) << " N/m\\n";
    return 0;
}
      
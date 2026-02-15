#include <iostream>
#include <string>

struct Hazard {
    std::string name;
    int severity;   // 1..4
    int probability; // 1..4
};

int riskIndex(const Hazard& h) {
    return h.severity * h.probability;
}

std::string riskLevel(int I) {
    if (I <= 4) return "low";
    if (I <= 8) return "medium";
    return "high";
}

int main() {
    Hazard hazards[] = {
        {"Pinch at wrist joint", 3, 3},
        {"Controller overtemperature", 2, 2},
        {"Unexpected fast motion", 4, 3}
    };

    for (const auto& h : hazards) {
        int I = riskIndex(h);
        std::cout << h.name
                  << ": I=" << I
                  << ", level=" << riskLevel(I)
                  << std::endl;
    }
    return 0;
}
      

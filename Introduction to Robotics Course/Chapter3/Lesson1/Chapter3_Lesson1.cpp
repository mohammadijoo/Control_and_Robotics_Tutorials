#include <iostream>
#include <string>

enum class Family { Manipulator, Mobile, Humanoid, Swarm, MobileManipulator, Other };

struct Robot {
    std::string name;
    int dof_base;
    int dof_joints;
    int n_agents;
    bool humanoid;

    int dof_total() const { return dof_base + dof_joints; }

    Family family() const {
        if (n_agents > 1) return Family::Swarm;
        if (humanoid) return Family::Humanoid;
        if (dof_base > 0 && dof_joints == 0) return Family::Mobile;
        if (dof_base == 0 && dof_joints > 0) return Family::Manipulator;
        if (dof_base > 0 && dof_joints > 0) return Family::MobileManipulator;
        return Family::Other;
    }
};

int main() {
    Robot arm{"6R arm", 0, 6, 1, false};
    Robot base{"Wheeled base", 3, 0, 1, false};
    Robot hum{"Humanoid", 6, 22, 1, true};
    Robot swarm{"Swarm", 3, 0, 100, false};

    Robot robots[] = {arm, base, hum, swarm};
    for (const auto& r : robots) {
        std::cout << r.name << " DoF=" << r.dof_total() << std::endl;
    }
}
      
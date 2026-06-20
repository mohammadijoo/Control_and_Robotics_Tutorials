
#include <iostream>
#include <vector>

struct Joint {
    int parent, child;
    int f; // DOF of joint
};

int spatialMobility(int N, const std::vector<Joint>& joints) {
    int J = static_cast<int>(joints.size());
    int sum_f = 0;
    for (const auto& j : joints) sum_f += j.f;
    return 6*(N - 1 - J) + sum_f;
}

int main() {
    // 3R planar arm is also spatially 3R with f=1 each
    int N = 4; // base + 3 links
    std::vector<Joint> joints = {
        {0,1,1}, {1,2,1}, {2,3,1}
    };
    std::cout << "Mobility = " << spatialMobility(N, joints) << std::endl;
    return 0;
}
      
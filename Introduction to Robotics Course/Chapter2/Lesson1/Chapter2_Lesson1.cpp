
#include 
#include 

int main() {
    std::vector N = {20, 60, 15, 45}; // tooth counts for gears 1..4
    double ratio = 1.0;
    int sign = 1;

    for (size_t k = 0; k + 1 < N.size(); ++k) {
        ratio *= static_cast(N[k]) / N[k+1];
        sign *= -1;
    }

    std::cout << "Speed ratio omega_m/omega_1 = "
              << sign * ratio << std::endl;
    return 0;
}
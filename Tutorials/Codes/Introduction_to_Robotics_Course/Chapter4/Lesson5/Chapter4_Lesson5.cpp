
#include <iostream>
#include <cmath>

double quant_step_rad(int bits){
    return 2.0 * M_PI / std::pow(2.0, bits);
}

int main(){
    int B = 17; // 17-bit absolute encoder
    double dtheta = quant_step_rad(B);
    double emax = dtheta / 2.0;
    std::cout << "Step (rad): " << dtheta << "\\n";
    std::cout << "Worst-case error (rad): " << emax << "\\n";
    return 0;
}
      
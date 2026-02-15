#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

int main() {
    double l1 = 1.0;
    double l2 = 0.7;

    double th1_min = -M_PI;
    double th1_max =  M_PI;
    double th2_min = -M_PI;
    double th2_max =  M_PI;

    int n1 = 300;
    int n2 = 300;

    std::ofstream out("workspace_2R.csv");
    out << "x,y,manip\n";

    for (int i = 0; i < n1; ++i) {
        double th1 = th1_min + (th1_max - th1_min) * i / (n1 - 1);
        for (int j = 0; j < n2; ++j) {
            double th2 = th2_min + (th2_max - th2_min) * j / (n2 - 1);

            double x = l1 * std::cos(th1) + l2 * std::cos(th1 + th2);
            double y = l1 * std::sin(th1) + l2 * std::sin(th1 + th2);

            double j11 = -l1 * std::sin(th1) - l2 * std::sin(th1 + th2);
            double j12 = -l2 * std::sin(th1 + th2);
            double j21 =  l1 * std::cos(th1) + l2 * std::cos(th1 + th2);
            double j22 =  l2 * std::cos(th1 + th2);

            Eigen::Matrix2d J;
            J(0,0) = j11; J(0,1) = j12;
            J(1,0) = j21; J(1,1) = j22;

            double manip = std::abs(J.determinant());
            out << x << "," << y << "," << manip << "\n";
        }
    }

    out.close();
    std::cout << "Saved workspace samples to workspace_2R.csv\n";
    return 0;
}
      

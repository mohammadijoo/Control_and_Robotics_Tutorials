#include <cmath>
#include <iostream>

double detJacobian2R(double theta1, double theta2,
                     double l1, double l2)
{
    return l1 * l2 * std::sin(theta2);
}

bool isKinematicSingular(double theta1, double theta2,
                         double l1, double l2,
                         double tol)
{
    double detJ = detJacobian2R(theta1, theta2, l1, l2);
    return std::fabs(detJ) < tol;
}

int main()
{
    double l1 = 1.0, l2 = 1.0;
    double theta1 = 0.0;
    double theta2 = 0.0; // singular (fully stretched)

    double detJ = detJacobian2R(theta1, theta2, l1, l2);
    std::cout << "detJ = " << detJ << std::endl;
    std::cout << "singular? "
              << (isKinematicSingular(theta1, theta2, l1, l2, 1e-6)
                  ? "yes" : "no")
              << std::endl;
    return 0;
}
      

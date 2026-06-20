\
/*
Chapter 3 — Lesson 4: Motion Primitives for Ground Vehicles (conceptual use)

Build:
  g++ -std=c++17 -O2 -o Chapter3_Lesson4 Chapter3_Lesson4.cpp
*/

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Primitive {
  std::string name;
  double v{0.0}, kappa{0.0}, T{0.0}, cost{0.0};
  double dx{0.0}, dy{0.0}, dtheta{0.0};
};

static void endpoint_constant_curvature(double v, double kappa, double T,
                                        double &dx, double &dy, double &dtheta) {
  dtheta = v * kappa * T;
  if (std::abs(kappa) < 1e-12) {
    dx = v * T; dy = 0.0;
  } else {
    dx = std::sin(dtheta) / kappa;
    dy = (1.0 - std::cos(dtheta)) / kappa;
  }
}

int main() {
  const double v = 0.6, kappa = 0.4, T = 1.2;
  double dx, dy, dtheta;
  endpoint_constant_curvature(v, kappa, T, dx, dy, dtheta);
  std::cout << "dx=" << dx << " dy=" << dy << " dtheta=" << dtheta << "\n";
  return 0;
}

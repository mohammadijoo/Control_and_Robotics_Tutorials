#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

using Scalar = std::complex<double>;

// Build monic polynomial from roots (highest degree first).
Eigen::VectorXcd polyFromRoots(const std::vector<Scalar>& roots) {
  if (roots.empty()) {
    Eigen::VectorXcd p(1);
    p(0) = Scalar(1.0, 0.0);
    return p;
  }
  Eigen::VectorXcd p(1);
  p(0) = Scalar(1.0, 0.0); // start with 1
  for (const auto& r : roots) {
    Eigen::VectorXcd q(p.size() + 1);
    q.setZero();
    for (int i = 0; i < p.size(); ++i) {
      q(i) += -r * p(i);
      q(i + 1) += p(i);
    }
    p = q;
  }
  return p;
}

// Differentiate polynomial p(s) (highest degree first).
Eigen::VectorXcd polyDeriv(const Eigen::VectorXcd& p) {
  int n = p.size() - 1;
  if (n <= 0) {
    Eigen::VectorXcd d(1);
    d(0) = Scalar(0.0, 0.0);
    return d;
  }
  Eigen::VectorXcd d(n);
  for (int i = 0; i < n; ++i) {
    d(i) = Scalar(double(n - i), 0.0) * p(i);
  }
  return d;
}

// Evaluate polynomial p(s) at s using Horner's method.
Scalar polyVal(const Eigen::VectorXcd& p, const Scalar& s) {
  Scalar y = Scalar(0.0, 0.0);
  for (int i = 0; i < p.size(); ++i) {
    y = y * s + p(i);
  }
  return y;
}

// Check if real point x lies on real-axis root-locus segment.
bool isOnRealAxisSegment(double x,
                         const std::vector<Scalar>& poles,
                         const std::vector<Scalar>& zeros,
                         double tol = 1e-8) {
  int count = 0;
  for (const auto& p : poles) {
    if (std::abs(p.imag()) < tol && p.real() > x) {
      ++count;
    }
  }
  for (const auto& z : zeros) {
    if (std::abs(z.imag()) < tol && z.real() > x) {
      ++count;
    }
  }
  return (count % 2) == 1;
}

int main() {
  // Example poles of a robotic joint open-loop model.
  std::vector<Scalar> poles = {
      Scalar(-2.0, 0.0), Scalar(-5.0, 0.0), Scalar(-20.0, 0.0)};
  std::vector<Scalar> zeros;  // no zeros

  Eigen::VectorXcd D = polyFromRoots(poles);
  Eigen::VectorXcd N = polyFromRoots(zeros);

  Eigen::VectorXcd Dp = polyDeriv(D);
  Eigen::VectorXcd Np = polyDeriv(N);

  // P(s) = N(s) D'(s) - D(s) N'(s)
  // Note: for empty zeros, N(s) = 1 and Np(s) = 0.
  int degP = std::max(Dp.size() + N.size() - 2, D.size() + Np.size() - 2);
  Eigen::VectorXcd P = Eigen::VectorXcd::Zero(degP + 1);

  // Convolution helper lambda.
  auto convolve = [](const Eigen::VectorXcd& a,
                     const Eigen::VectorXcd& b) {
    Eigen::VectorXcd c(a.size() + b.size() - 1);
    c.setZero();
    for (int i = 0; i < a.size(); ++i)
      for (int j = 0; j < b.size(); ++j)
        c(i + j) += a(i) * b(j);
    return c;
  };

  Eigen::VectorXcd NDp = convolve(N, Dp);
  Eigen::VectorXcd DNp = convolve(D, Np);
  // P = NDp - DNp
  int offset = P.size() - NDp.size();
  for (int i = 0; i < NDp.size(); ++i) P(i + offset) += NDp(i);
  offset = P.size() - DNp.size();
  for (int i = 0; i < DNp.size(); ++i) P(i + offset) -= DNp(i);

  // Solve P(s) = 0 using Eigen polynomial solver.
  Eigen::PolynomialSolver<std::complex<double>, Eigen::Dynamic> solver;
  solver.compute(P);
  auto rootsP = solver.roots();

  std::cout << "Candidate breakaway/break-in points:\n";
  std::vector<double> realCandidates;
  for (int i = 0; i < rootsP.size(); ++i) {
    Scalar r = rootsP(i);
    if (std::abs(r.imag()) < 1e-6) {
      double x = r.real();
      realCandidates.push_back(x);
      std::cout << "  s = " << x << "\n";
    }
  }

  std::cout << "On real-axis root-locus segments:\n";
  for (double x : realCandidates) {
    if (isOnRealAxisSegment(x, poles, zeros)) {
      std::cout << "  s = " << x << " is on the root locus (real axis).\n";
    }
  }

  return 0;
}

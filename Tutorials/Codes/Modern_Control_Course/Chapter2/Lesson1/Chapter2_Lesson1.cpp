#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct RrefResult {
  Eigen::MatrixXd R;
  std::vector<int> pivots;
};

RrefResult rref(const Eigen::MatrixXd& A, double tol = 1e-10) {
  Eigen::MatrixXd R = A;
  const int m = (int)R.rows();
  const int n = (int)R.cols();

  std::vector<int> pivots;
  int row = 0;

  for (int col = 0; col < n && row < m; ++col) {
    // Partial pivoting: pick row with largest magnitude in column
    int pivot = row;
    double maxAbs = std::abs(R(row, col));
    for (int r = row + 1; r < m; ++r) {
      double val = std::abs(R(r, col));
      if (val > maxAbs) {
        maxAbs = val;
        pivot = r;
      }
    }
    if (maxAbs < tol) continue;

    if (pivot != row) R.row(row).swap(R.row(pivot));

    // Normalize pivot row
    double piv = R(row, col);
    R.row(row) /= piv;

    // Eliminate other rows
    for (int r = 0; r < m; ++r) {
      if (r == row) continue;
      double factor = R(r, col);
      R.row(r) -= factor * R.row(row);
    }

    pivots.push_back(col);
    ++row;
  }

  // Zero out tiny noise
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      if (std::abs(R(i, j)) < tol) R(i, j) = 0.0;

  return {R, pivots};
}

int main() {
  Eigen::MatrixXd V(4,4);
  V << 1,2,0,1,
       0,1,1,1,
       1,3,1,2,
       0,0,1,1;

  auto res = rref(V);
  std::cout << "Pivot columns: ";
  for (int c : res.pivots) std::cout << c << " ";
  std::cout << "\nDimension of span: " << res.pivots.size() << "\n";

  std::cout << "RREF:\n" << res.R << "\n";

  // Extract basis from original columns
  Eigen::MatrixXd B(V.rows(), (int)res.pivots.size());
  for (int i = 0; i < (int)res.pivots.size(); ++i) {
    B.col(i) = V.col(res.pivots[i]);
  }
  std::cout << "Basis columns:\n" << B << "\n";
  return 0;
}
      

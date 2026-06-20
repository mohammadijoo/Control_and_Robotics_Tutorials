/*
Chapter26_Lesson3.cpp
SISO augmented-state pole placement with integral action using Eigen.

Build example:
  g++ -std=c++17 Chapter26_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter26_Lesson3

Dependency:
  Eigen 3.x, header-only: https://eigen.tuxfamily.org
*/

#include &lt;Eigen/Dense&gt;
#include &lt;iostream&gt;
#include &lt;vector&gt;

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    int n = static_cast&lt;int&gt;(A.rows());
    int m = static_cast&lt;int&gt;(B.cols());
    MatrixXd Co(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k &lt; n; ++k) {
        Co.block(0, k * m, n, m) = Ak * B;
        Ak = Ak * A;
    }
    return Co;
}

std::vector&lt;double&gt; polynomialFromRoots(const std::vector&lt;double&gt;& roots) {
    // Returns coefficients [1, a_{n-1}, ..., a_0] for prod(s - root_i).
    std::vector&lt;double&gt; coeff{1.0};
    for (double r : roots) {
        std::vector&lt;double&gt; next(coeff.size() + 1, 0.0);
        for (std::size_t i = 0; i &lt; coeff.size(); ++i) {
            next[i] += coeff[i];
            next[i + 1] += -r * coeff[i];
        }
        coeff = next;
    }
    return coeff;
}

RowVectorXd ackermannSISO(const MatrixXd& A, const MatrixXd& B,
                         const std::vector&lt;double&gt;& desiredPoles) {
    int n = static_cast&lt;int&gt;(A.rows());
    MatrixXd Co = controllabilityMatrix(A, B);
    MatrixXd CoInv = Co.inverse();

    std::vector&lt;double&gt; coeff = polynomialFromRoots(desiredPoles);
    MatrixXd phiA = MatrixXd::Zero(n, n);
    MatrixXd Ak = MatrixXd::Identity(n, n);

    // phi(A) = A^n + a_{n-1}A^{n-1} + ... + a_0 I.
    std::vector&lt;MatrixXd&gt; powers(n + 1);
    powers[0] = MatrixXd::Identity(n, n);
    for (int k = 1; k &lt;= n; ++k) {
        powers[k] = powers[k - 1] * A;
    }
    for (int power = n; power &gt;= 0; --power) {
        double c = coeff[n - power];
        phiA += c * powers[power];
    }

    RowVectorXd en = RowVectorXd::Zero(n);
    en(n - 1) = 1.0;
    return en * CoInv * phiA;
}

int main() {
    MatrixXd A(2, 2);
    A &lt;&lt; 0.0, 1.0,
        -2.0, -0.6;
    MatrixXd B(2, 1);
    B &lt;&lt; 0.0,
         1.0;
    MatrixXd C(1, 2);
    C &lt;&lt; 1.0, 0.0;
    MatrixXd D(1, 1);
    D &lt;&lt; 0.0;

    int n = 2;
    int p = 1;
    MatrixXd Aaug = MatrixXd::Zero(n + p, n + p);
    Aaug.block(0, 0, n, n) = A;
    Aaug.block(n, 0, p, n) = -C;

    MatrixXd Baug(n + p, 1);
    Baug.block(0, 0, n, 1) = B;
    Baug.block(n, 0, p, 1) = -D;

    MatrixXd Co = controllabilityMatrix(Aaug, Baug);
    std::cout &lt;&lt; "Controllability rank estimate: "
              &lt;&lt; Co.fullPivLu().rank() &lt;&lt; " of " &lt;&lt; Aaug.rows() &lt;&lt; "\n";

    std::vector&lt;double&gt; poles{-2.0, -2.5, -3.0};
    RowVectorXd Kaug = ackermannSISO(Aaug, Baug, poles);
    std::cout &lt;&lt; "K_aug = " &lt;&lt; Kaug &lt;&lt; "\n";
    std::cout &lt;&lt; "Kx    = " &lt;&lt; Kaug.leftCols(n) &lt;&lt; "\n";
    std::cout &lt;&lt; "Ki    = " &lt;&lt; Kaug.rightCols(p) &lt;&lt; "\n";

    MatrixXd Acl = Aaug - Baug * Kaug;
    std::cout &lt;&lt; "A_cl =\n" &lt;&lt; Acl &lt;&lt; "\n";
    return 0;
}

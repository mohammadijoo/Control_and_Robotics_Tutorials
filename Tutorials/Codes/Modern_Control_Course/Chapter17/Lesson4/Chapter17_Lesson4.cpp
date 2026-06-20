/*
Chapter17_Lesson4.cpp
Transformations Between Physical and Modal Coordinates

This compact C++ example uses a 2x2 system with known real eigenvectors.
A = [[0, 1], [-2, -3]] has eigenvalues -1 and -2.
With T = [[1, 1], [-1, -2]], Lambda = T^{-1} A T.
*/

#include &lt;iostream&gt;
#include &lt;array&gt;
#include &lt;iomanip&gt;

using Mat2 = std::array&lt;std::array&lt;double, 2&gt;, 2&gt;;
using Vec2 = std::array&lt;double, 2&gt;;

Mat2 multiply(const Mat2& A, const Mat2& B) {
    Mat2 C{};
    for (int i = 0; i &lt; 2; ++i)
        for (int j = 0; j &lt; 2; ++j)
            for (int k = 0; k &lt; 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

Vec2 multiply(const Mat2& A, const Vec2& x) {
    Vec2 y{};
    for (int i = 0; i &lt; 2; ++i)
        for (int k = 0; k &lt; 2; ++k)
            y[i] += A[i][k] * x[k];
    return y;
}

Mat2 inverse2x2(const Mat2& A) {
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (det &gt; -1e-12 && det &lt; 1e-12) {
        throw std::runtime_error("Matrix is singular or nearly singular.");
    }
    return Mat2{{
        {{ A[1][1] / det, -A[0][1] / det }},
        {{-A[1][0] / det,  A[0][0] / det }}
    }};
}

void printMat(const char* name, const Mat2& A) {
    std::cout &lt;&lt; name &lt;&lt; "\n";
    for (int i = 0; i &lt; 2; ++i) {
        std::cout &lt;&lt; "  ";
        for (int j = 0; j &lt; 2; ++j)
            std::cout &lt;&lt; std::setw(10) &lt;&lt; A[i][j] &lt;&lt; " ";
        std::cout &lt;&lt; "\n";
    }
}

void printVec(const char* name, const Vec2& x) {
    std::cout &lt;&lt; name &lt;&lt; " = [" &lt;&lt; x[0] &lt;&lt; ", " &lt;&lt; x[1] &lt;&lt; "]^T\n";
}

int main() {
    Mat2 A{{ {{0.0, 1.0}}, {{-2.0, -3.0}} }};
    Mat2 T{{ {{1.0, 1.0}}, {{-1.0, -2.0}} }};      // eigenvectors as columns
    Mat2 Tinv = inverse2x2(T);
    Mat2 Lambda = multiply(multiply(Tinv, A), T);

    Vec2 B{0.0, 1.0};
    Vec2 Bm = multiply(Tinv, B);

    // C = [1, 0], so Cm = C T is the first row of T.
    Vec2 Cm{T[0][0], T[0][1]};

    Vec2 x0{2.0, -1.0};
    Vec2 z0 = multiply(Tinv, x0);
    Vec2 x0Recovered = multiply(T, z0);

    std::cout &lt;&lt; std::fixed &lt;&lt; std::setprecision(5);
    printMat("A:", A);
    printMat("T:", T);
    printMat("Tinv:", Tinv);
    printMat("Lambda = Tinv A T:", Lambda);
    printVec("Bm = Tinv B", Bm);
    printVec("Cm = C T", Cm);
    printVec("z0 = Tinv x0", z0);
    printVec("T z0", x0Recovered);

    std::cout &lt;&lt; "\nInterpretation: z1 and z2 are modal amplitudes associated with eigenvalues -1 and -2.\n";
    return 0;
}

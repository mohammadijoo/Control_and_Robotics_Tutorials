// Requires (typical):
//   org.ejml:ejml-all
//   org.apache.commons:commons-math3

import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

public class PoleZeroFromStateSpaceSISO2 {
  // For the 2x2 example, compute:
  // Delta(s) = det(sI-A) = s^2 - tr(A)s + det(A)
  // and N(s) = det(P(s)) = det([[sI-A, -B],[C, D]]) (3x3 determinant)
  public static void main(String[] args) {
    double[][] A = { {0, 1},
                    {-2, -3} };
    double[] B = {0, 1};
    double[] C = {1, 1};
    double D = 0;

    // Delta(s) = det([[s-0, -1],[2, s+3]]) = (s)(s+3)+2 = s^2+3s+2
    double[] delta = {2, 3, 1}; // 2 + 3 s + 1 s^2

    // Build det(P(s)) directly for this specific structure:
    // P(s) = [[ s, -1, -B1],
    //         [ 2, s+3, -B2],
    //         [ C1, C2, D ]]
    // Here B1=0, B2=1, C1=1, C2=1, D=0.
    // Compute determinant symbolically:
    // det(P) = s*det([[s+3, -1],[1, 0]]) - (-1)*det([[2, -1],[1, 0]]) + 0*...
    //        = s*((s+3)*0 - (-1)*1) + 1*(2*0 - (-1)*1)
    //        = s*(1) + 1*(1) = s+1
    double[] num = {1, 1}; // 1 + 1 s

    // Roots using LaguerreSolver expects coefficients highest order first
    LaguerreSolver solver = new LaguerreSolver();

    Complex[] poles = solver.solveAllComplex(new double[]{delta[2], delta[1], delta[0]}, 0);
    Complex[] zeros = solver.solveAllComplex(new double[]{num[1], num[0]}, 0);

    System.out.println("Poles (roots of Delta):");
    for (Complex c : poles) System.out.println("  " + c);

    System.out.println("Zeros (roots of N):");
    for (Complex c : zeros) System.out.println("  " + c);

    System.out.println("Note: cancellation may remove common pole/zero factors in the reduced G(s).");
  }
}

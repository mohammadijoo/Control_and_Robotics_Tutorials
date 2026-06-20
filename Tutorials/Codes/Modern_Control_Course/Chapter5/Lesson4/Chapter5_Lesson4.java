import java.util.Arrays;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class PhaseVariableSS {

  public static class StateSpace {
    public final RealMatrix A, B, C, D;
    public StateSpace(RealMatrix A, RealMatrix B, RealMatrix C, RealMatrix D) {
      this.A = A; this.B = B; this.C = C; this.D = D;
    }
  }

  // a: [a0, a1, ..., a_{n-1}], b: [b1, ..., bm]
  public static StateSpace phaseVariable(double[] a, double[] b, RealMatrix C, RealMatrix D) {
    int n = a.length;
    int m = b.length;

    RealMatrix A = MatrixUtils.createRealMatrix(n, n);
    for (int i = 0; i < n - 1; i++) {
      A.setEntry(i, i + 1, 1.0);
    }
    for (int j = 0; j < n; j++) {
      A.setEntry(n - 1, j, -a[j]);
    }

    RealMatrix B = MatrixUtils.createRealMatrix(n, m);
    for (int j = 0; j < m; j++) {
      B.setEntry(n - 1, j, b[j]);
    }

    return new StateSpace(A, B, C, D);
  }

  public static void main(String[] args) {
    // Example 2
    double[] a = new double[] {2.0, 3.0, 1.0};  // a0, a1, a2
    double[] b = new double[] {5.0, -1.0};      // b1, b2

    RealMatrix C = MatrixUtils.createRealMatrix(new double[][] {
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0}
    });
    RealMatrix D = MatrixUtils.createRealMatrix(new double[][] {
      {0.0, 0.0},
      {0.0, 0.0}
    });

    StateSpace sys = phaseVariable(a, b, C, D);

    System.out.println("A=\n" + Arrays.deepToString(sys.A.getData()));
    System.out.println("B=\n" + Arrays.deepToString(sys.B.getData()));
    System.out.println("C=\n" + Arrays.deepToString(sys.C.getData()));
    System.out.println("D=\n" + Arrays.deepToString(sys.D.getData()));
  }
}
      

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class RealizationEquivalence {
  static double evalG(double s, DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, double D) {
    int n = A.numRows;
    DMatrixRMaj I = CommonOps_DDRM.identity(n);

    // M = s I - A
    DMatrixRMaj M = new DMatrixRMaj(n, n);
    CommonOps_DDRM.scale(s, I, M);
    CommonOps_DDRM.subtractEquals(M, A);

    // Solve M x = B
    DMatrixRMaj x = new DMatrixRMaj(n, 1);
    CommonOps_DDRM.solve(M, B, x);

    // val = C x + D
    DMatrixRMaj Cx = new DMatrixRMaj(1, 1);
    CommonOps_DDRM.mult(C, x, Cx);
    return Cx.get(0, 0) + D;
  }

  public static void main(String[] args) {
    // A, B, C, D for G(s) = (s+2) / (s^2 + 3 s + 2)
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
        {0.0, 1.0},
        {-2.0, -3.0}
    });
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{
        {0.0},
        {1.0}
    });
    DMatrixRMaj C = new DMatrixRMaj(new double[][]{
        {2.0, 1.0}
    });
    double D = 0.0;

    // Similarity transform z = T x
    DMatrixRMaj T = new DMatrixRMaj(new double[][]{
        {1.0, 1.0},
        {0.0, 1.0}
    });
    DMatrixRMaj Ti = new DMatrixRMaj(2, 2);
    CommonOps_DDRM.invert(T, Ti);

    DMatrixRMaj A2 = new DMatrixRMaj(2, 2);
    DMatrixRMaj temp = new DMatrixRMaj(2, 2);
    CommonOps_DDRM.mult(T, A, temp);
    CommonOps_DDRM.mult(temp, Ti, A2);

    DMatrixRMaj B2 = new DMatrixRMaj(2, 1);
    CommonOps_DDRM.mult(T, B, B2);

    DMatrixRMaj C2 = new DMatrixRMaj(1, 2);
    CommonOps_DDRM.mult(C, Ti, C2);

    double[] tests = new double[]{1.0, 2.0, 3.0};
    for (double s : tests) {
      double g1 = evalG(s, A, B, C, D);
      double g2 = evalG(s, A2, B2, C2, D);
      System.out.println("s=" + s + "  G1(s)=" + g1 + "  G2(s)=" + g2);
    }
  }
}
      

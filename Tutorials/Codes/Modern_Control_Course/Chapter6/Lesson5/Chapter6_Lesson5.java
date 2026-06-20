import org.ejml.data.ZMatrixRMaj;
import org.ejml.dense.row.CommonOps_ZDRM;
import org.ejml.dense.row.NormOps_ZDRM;

public class FreqRespCheck {

  // Computes scalar G(jw)=C*(jwI - A)^(-1)*B + D for SISO
  static double[] freqresp(double[][] A, double[] B, double[] C, double D, double w) {
    int n = A.length;

    // Build M = jwI - A as complex matrix
    ZMatrixRMaj M = new ZMatrixRMaj(n, n);
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        double re = -A[i][j];
        double im = 0.0;
        if (i == j) im += w; // add jw on diagonal
        M.set(i, j, re, im);
      }
    }

    // Build complex vector Bb
    ZMatrixRMaj Bb = new ZMatrixRMaj(n, 1);
    for (int i = 0; i < n; i++) Bb.set(i, 0, B[i], 0.0);

    // Solve M x = B
    ZMatrixRMaj x = new ZMatrixRMaj(n, 1);
    CommonOps_ZDRM.solve(M, Bb, x);

    // y = C x
    double yRe = 0.0, yIm = 0.0;
    for (int i = 0; i < n; i++) {
      double xr = x.getReal(i, 0);
      double xi = x.getImag(i, 0);
      yRe += C[i] * xr;
      yIm += C[i] * xi;
    }

    // Add D
    yRe += D;
    return new double[]{yRe, yIm};
  }

  public static void main(String[] args) {
    // Minimal: xdot=-2x+u, y=x
    double[][] A1 = {{-2.0}};
    double[] B1 = {1.0};
    double[] C1 = {1.0};
    double D1 = 0.0;

    // Nonminimal: add hidden state xh, xhdot=-1*xh, no coupling
    double[][] A2 = { {-2.0, 0.0},{0.0, -1.0} };
    double[] B2 = {1.0, 0.0};
    double[] C2 = {1.0, 0.0};
    double D2 = 0.0;

    double[] ws = {0.01, 0.1, 1.0, 10.0, 100.0};
    for (double w : ws) {
      double[] Gm = freqresp(A1, B1, C1, D1, w);
      double[] Gn = freqresp(A2, B2, C2, D2, w);
      System.out.printf("w=%f  G_min=(%e,%e)  G_nonmin=(%e,%e)  diff=(%e,%e)%n",
          w, Gm[0], Gm[1], Gn[0], Gn[1], (Gm[0]-Gn[0]), (Gm[1]-Gn[1]));
    }
  }
}
      

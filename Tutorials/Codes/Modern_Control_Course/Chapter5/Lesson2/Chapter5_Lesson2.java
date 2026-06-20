import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.function.DoubleFunction;

public class CompanionSS {

  static class SS {
    DMatrixRMaj A;   // n x n
    DMatrixRMaj B;   // n x 1
    DMatrixRMaj C;   // 1 x n
    double D;        // scalar
  }

  static SS companionFromOde(double[] a, double b) {
    int n = a.length;
    SS sys = new SS();
    sys.A = new DMatrixRMaj(n, n);
    sys.B = new DMatrixRMaj(n, 1);
    sys.C = new DMatrixRMaj(1, n);
    sys.D = 0.0;

    // superdiagonal ones
    for (int i = 0; i < n - 1; i++) sys.A.set(i, i + 1, 1.0);

    // last row = -a
    for (int j = 0; j < n; j++) sys.A.set(n - 1, j, -a[j]);

    sys.B.set(n - 1, 0, b);
    sys.C.set(0, 0, 1.0);
    return sys;
  }

  static DMatrixRMaj f(SS sys, double t, DMatrixRMaj x, DoubleFunction<Double> uOfT) {
    DMatrixRMaj Ax = new DMatrixRMaj(x.numRows, 1);
    CommonOps_DDRM.mult(sys.A, x, Ax);

    DMatrixRMaj Bu = new DMatrixRMaj(x.numRows, 1);
    CommonOps_DDRM.scale(uOfT.apply(t), sys.B, Bu);

    CommonOps_DDRM.addEquals(Ax, Bu);
    return Ax; // dx/dt
  }

  static DMatrixRMaj rk4Step(SS sys, double t, DMatrixRMaj x, double h, DoubleFunction<Double> uOfT) {
    DMatrixRMaj k1 = f(sys, t, x, uOfT);

    DMatrixRMaj x2 = x.copy();
    CommonOps_DDRM.addEquals(x2, 0.5*h, k1);
    DMatrixRMaj k2 = f(sys, t + 0.5*h, x2, uOfT);

    DMatrixRMaj x3 = x.copy();
    CommonOps_DDRM.addEquals(x3, 0.5*h, k2);
    DMatrixRMaj k3 = f(sys, t + 0.5*h, x3, uOfT);

    DMatrixRMaj x4 = x.copy();
    CommonOps_DDRM.addEquals(x4, h, k3);
    DMatrixRMaj k4 = f(sys, t + h, x4, uOfT);

    DMatrixRMaj out = x.copy();
    // out = x + (h/6)*(k1 + 2k2 + 2k3 + k4)
    CommonOps_DDRM.addEquals(out, h/6.0, k1);
    CommonOps_DDRM.addEquals(out, h/3.0, k2);
    CommonOps_DDRM.addEquals(out, h/3.0, k3);
    CommonOps_DDRM.addEquals(out, h/6.0, k4);
    return out;
  }

  public static void main(String[] args) {
    // Example: y'' + 3 y' + 2 y = u
    SS sys = companionFromOde(new double[]{2.0, 3.0}, 1.0);

    DoubleFunction<Double> uOfT = (double t) -> (t >= 0.0) ? 1.0 : 0.0;

    DMatrixRMaj x = new DMatrixRMaj(2, 1);
    x.set(0, 0, 0.0); // y(0)
    x.set(1, 0, 0.0); // y'(0)

    double t = 0.0, tf = 5.0, h = 1e-3;
    int N = (int)Math.round((tf - t)/h);

    for (int k = 0; k < N; k++) {
      x = rk4Step(sys, t, x, h, uOfT);
      t += h;
    }

    // y = C x + D u
    DMatrixRMaj y = new DMatrixRMaj(1, 1);
    CommonOps_DDRM.mult(sys.C, x, y);
    y.set(0, 0, y.get(0, 0) + sys.D * uOfT.apply(t));
    System.out.println("Final y(tf) = " + y.get(0, 0));
  }
}

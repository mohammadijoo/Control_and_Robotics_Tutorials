import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class MimoStateSpaceEuler {
  public static void main(String[] args) {
    // Matrices from Section 5
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {0.0, 1.0},
      {-2.0, -3.0}
    });
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{
      {0.0, 0.0},
      {4.0, -1.0}
    });
    DMatrixRMaj C = new DMatrixRMaj(new double[][]{
      {1.0, 0.0},
      {0.0, 1.0}
    });
    DMatrixRMaj D = new DMatrixRMaj(new double[][]{
      {0.0, 0.0},
      {0.0, 0.5}
    });

    double T = 10.0;
    double dt = 0.005;
    int N = (int)(T / dt) + 1;

    DMatrixRMaj x = new DMatrixRMaj(2, 1); // zero init

    DMatrixRMaj u = new DMatrixRMaj(2, 1);
    DMatrixRMaj Ax = new DMatrixRMaj(2, 1);
    DMatrixRMaj Bu = new DMatrixRMaj(2, 1);
    DMatrixRMaj xdot = new DMatrixRMaj(2, 1);

    DMatrixRMaj y = new DMatrixRMaj(2, 1);
    DMatrixRMaj Cx = new DMatrixRMaj(2, 1);
    DMatrixRMaj Du = new DMatrixRMaj(2, 1);

    for (int k = 0; k < N; k++) {
      double t = k * dt;

      // Inputs
      u.set(0, 0, 1.0);
      u.set(1, 0, Math.exp(-0.7 * t));

      // y = Cx + Du
      CommonOps_DDRM.mult(C, x, Cx);
      CommonOps_DDRM.mult(D, u, Du);
      CommonOps_DDRM.add(Cx, Du, y);

      if (k % 400 == 0) {
        System.out.printf("t=%.3f  y1=%.6f  y2=%.6f  x1=%.6f  x2=%.6f%n",
          t, y.get(0,0), y.get(1,0), x.get(0,0), x.get(1,0));
      }

      // xdot = Ax + Bu
      CommonOps_DDRM.mult(A, x, Ax);
      CommonOps_DDRM.mult(B, u, Bu);
      CommonOps_DDRM.add(Ax, Bu, xdot);

      // Euler update: x = x + dt * xdot
      x.add(0, 0, dt * xdot.get(0,0));
      x.add(1, 0, dt * xdot.get(1,0));
    }
  }
}
      

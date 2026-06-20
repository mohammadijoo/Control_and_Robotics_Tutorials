import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class MassSpringDamperSim {
  public static void main(String[] args) {
    double m = 1.0, b = 0.4, k = 4.0;

    // A (2x2), B (2x1), C (1x2)
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {0.0, 1.0},
      {-k/m, -b/m}
    });
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{
      {0.0},
      {1.0/m}
    });
    DMatrixRMaj C = new DMatrixRMaj(new double[][]{
      {1.0, 0.0}
    });

    double dt = 1e-3;
    double T  = 10.0;
    int N = (int)(T / dt);

    DMatrixRMaj x = new DMatrixRMaj(new double[][]{ {0.0},{0.0} });
    DMatrixRMaj Ax = new DMatrixRMaj(2,1);
    DMatrixRMaj xdot = new DMatrixRMaj(2,1);

    for (int i = 0; i < N; i++) {
      double t = i * dt;
      double u = 1.0; // step input

      CommonOps_DDRM.mult(A, x, Ax);           // Ax
      xdot.set(0, 0, Ax.get(0,0) + B.get(0,0) * u);
      xdot.set(1, 0, Ax.get(1,0) + B.get(1,0) * u);

      // x = x + dt*xdot (Euler)
      x.set(0, 0, x.get(0,0) + dt * xdot.get(0,0));
      x.set(1, 0, x.get(1,0) + dt * xdot.get(1,0));

      if (i % 2000 == 0) {
        double y = C.get(0,0) * x.get(0,0) + C.get(0,1) * x.get(1,0);
        System.out.println("t=" + t + "  q=" + x.get(0,0) + "  qdot=" + x.get(1,0) + "  y=" + y);
      }
    }

    System.out.println("Final state: q=" + x.get(0,0) + ", qdot=" + x.get(1,0));
  }
}
      

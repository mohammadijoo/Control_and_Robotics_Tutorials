import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class LTISim {

  static DMatrixRMaj xdot(DMatrixRMaj A, DMatrixRMaj x, DMatrixRMaj B, DMatrixRMaj u) {
    DMatrixRMaj Ax = new DMatrixRMaj(A.numRows, 1);
    DMatrixRMaj Bu = new DMatrixRMaj(B.numRows, 1);
    CommonOps_DDRM.mult(A, x, Ax);
    CommonOps_DDRM.mult(B, u, Bu);
    CommonOps_DDRM.addEquals(Ax, Bu);
    return Ax;
  }

  static DMatrixRMaj rk4Step(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj x, DMatrixRMaj u, double h) {
    DMatrixRMaj k1 = xdot(A, x, B, u);

    DMatrixRMaj x2 = x.copy(); CommonOps_DDRM.addEquals(x2, 0.5*h, k1);
    DMatrixRMaj k2 = xdot(A, x2, B, u);

    DMatrixRMaj x3 = x.copy(); CommonOps_DDRM.addEquals(x3, 0.5*h, k2);
    DMatrixRMaj k3 = xdot(A, x3, B, u);

    DMatrixRMaj x4 = x.copy(); CommonOps_DDRM.addEquals(x4, h, k3);
    DMatrixRMaj k4 = xdot(A, x4, B, u);

    DMatrixRMaj sum = k1.copy();
    CommonOps_DDRM.addEquals(sum, 2.0, k2);
    CommonOps_DDRM.addEquals(sum, 2.0, k3);
    CommonOps_DDRM.addEquals(sum, 1.0, k4);

    DMatrixRMaj xNext = x.copy();
    CommonOps_DDRM.addEquals(xNext, h/6.0, sum);
    return xNext;
  }

  public static void main(String[] args) {
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{ {0,1},{-2,-3} });
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{ {0},{1} });
    DMatrixRMaj C = new DMatrixRMaj(new double[][]{ {1,0} });
    DMatrixRMaj D = new DMatrixRMaj(new double[][]{ {0} });

    double t0 = 0.0, tf = 5.0, h = 1e-3;
    int N = (int)((tf - t0)/h);

    DMatrixRMaj x = new DMatrixRMaj(new double[][]{ {0.5},{0.0} });
    DMatrixRMaj u = new DMatrixRMaj(new double[][]{ {1.0} });

    for (int k = 0; k < N; k++) {
      x = rk4Step(A, B, x, u, h);
    }

    // y = Cx + Du
    DMatrixRMaj y = new DMatrixRMaj(1,1);
    CommonOps_DDRM.mult(C, x, y);
    DMatrixRMaj Du = new DMatrixRMaj(1,1);
    CommonOps_DDRM.mult(D, u, Du);
    CommonOps_DDRM.addEquals(y, Du);

    System.out.println("x(tf)=\n" + x);
    System.out.println("y(tf)=\n" + y);
  }
}
      

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class StateScalingDemo {
  public static void main(String[] args) {
    // A (3x3)
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
      {-1.0, -10.0, -1000.0}
    });

    // B (3x1), C (1x3), D (1x1)
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{ {0.0},{0.0},{1.0} });
    DMatrixRMaj C = new DMatrixRMaj(new double[][]{ {1.0,0.0,0.0} });
    DMatrixRMaj D = new DMatrixRMaj(new double[][]{ {0.0} });

    double omega = 10.0;
    DMatrixRMaj S = new DMatrixRMaj(3,3);
    CommonOps_DDRM.setIdentity(S);
    S.set(1,1, omega);
    S.set(2,2, omega*omega);

    DMatrixRMaj Sinv = new DMatrixRMaj(3,3);
    CommonOps_DDRM.invert(S, Sinv);

    // As = Sinv * A * S
    DMatrixRMaj tmp = new DMatrixRMaj(3,3);
    DMatrixRMaj As  = new DMatrixRMaj(3,3);
    CommonOps_DDRM.mult(Sinv, A, tmp);
    CommonOps_DDRM.mult(tmp, S, As);

    // Bs = Sinv * B
    DMatrixRMaj Bs = new DMatrixRMaj(3,1);
    CommonOps_DDRM.mult(Sinv, B, Bs);

    // Cs = C * S
    DMatrixRMaj Cs = new DMatrixRMaj(1,3);
    CommonOps_DDRM.mult(C, S, Cs);

    System.out.println("As =\n" + As);
    System.out.println("Bs =\n" + Bs);
    System.out.println("Cs =\n" + Cs);
    System.out.println("D  =\n" + D);
  }
}

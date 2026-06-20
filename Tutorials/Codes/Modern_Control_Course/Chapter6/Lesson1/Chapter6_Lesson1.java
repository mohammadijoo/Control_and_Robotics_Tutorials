import org.ejml.data.ZMatrixRMaj;
import org.ejml.dense.row.CommonOps_ZDRM;

public class TransferFromStateSpace {
  public static void main(String[] args) {
    // Example: n=2, SISO
    // A, B, C, D are real but stored in complex form (imag part = 0)
    ZMatrixRMaj A = new ZMatrixRMaj(2,2);
    A.set(0,0, 0.0, 0.0);  A.set(0,1, 1.0, 0.0);
    A.set(1,0,-2.0, 0.0);  A.set(1,1,-3.0, 0.0);

    ZMatrixRMaj B = new ZMatrixRMaj(2,1);
    B.set(0,0, 0.0, 0.0);
    B.set(1,0, 1.0, 0.0);

    ZMatrixRMaj C = new ZMatrixRMaj(1,2);
    C.set(0,0, 1.0, 0.0);
    C.set(0,1, 0.0, 0.0);

    ZMatrixRMaj D = new ZMatrixRMaj(1,1);
    D.set(0,0, 0.0, 0.0);

    // s = j*2
    double sRe = 0.0;
    double sIm = 2.0;

    // M = sI - A
    ZMatrixRMaj M = new ZMatrixRMaj(2,2);
    CommonOps_ZDRM.setIdentity(M);
    // scale I by s
    CommonOps_ZDRM.scale(sRe, sIm, M);
    // M = M - A
    CommonOps_ZDRM.subtractEquals(M, A);

    // Solve M X = B
    ZMatrixRMaj X = new ZMatrixRMaj(2,1);
    if (!CommonOps_ZDRM.solve(M, B, X)) {
      throw new RuntimeException("Solve failed (matrix may be singular for this s).");
    }

    // G = C X + D
    ZMatrixRMaj G = new ZMatrixRMaj(1,1);
    CommonOps_ZDRM.mult(C, X, G);
    CommonOps_ZDRM.addEquals(G, D);

    System.out.println("G(j2) = " + G.getReal(0,0) + " + j" + G.getImag(0,0));
  }
}
      

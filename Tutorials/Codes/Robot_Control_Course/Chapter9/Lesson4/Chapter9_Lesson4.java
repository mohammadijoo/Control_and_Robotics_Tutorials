
import org.ejml.simple.SimpleMatrix;

public class LQR {
    public static class LQRResult {
        public SimpleMatrix[] K;
    }

    public static LQRResult finiteHorizonLQR(SimpleMatrix[] A,
                                             SimpleMatrix[] B,
                                             SimpleMatrix Q,
                                             SimpleMatrix R,
                                             SimpleMatrix P_N) {
        int N = A.length;
        int nx = Q.numRows();
        int nu = R.numRows();

        SimpleMatrix[] P = new SimpleMatrix[N + 1];
        P[N] = P_N.copy();

        SimpleMatrix[] K = new SimpleMatrix[N];

        for (int k = N - 1; k >= 0; --k) {
            SimpleMatrix Ak = A[k];
            SimpleMatrix Bk = B[k];
            SimpleMatrix Pkp1 = P[k + 1];

            SimpleMatrix S = R.plus(Bk.transpose().mult(Pkp1).mult(Bk));
            SimpleMatrix Kk = S.invert().negative().mult(Bk.transpose()).mult(Pkp1).mult(Ak);

            SimpleMatrix Pk = Q.plus(
                    Ak.transpose().mult(Pkp1).mult(Ak))
                .plus(Ak.transpose().mult(Pkp1).mult(Bk).mult(Kk))
                .plus(Kk.transpose().mult(Bk.transpose()).mult(Pkp1).mult(Ak))
                .plus(Kk.transpose().mult(R).mult(Kk));

            K[k] = Kk;
            P[k] = Pk;
        }

        LQRResult result = new LQRResult();
        result.K = K;
        return result;
    }

    // Real-time loop snippet (pseudo-code):
    // SimpleMatrix x = ...;      // current state
    // SimpleMatrix xRef = ...;   // reference
    // SimpleMatrix u = K[0].mult(x.minus(xRef));
}

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;

public class MatrixExponentialLesson {

    // Educational series: exp(A t) ~= sum_{k=0}^K (A t)^k / k!
    public static DMatrixRMaj expmSeries(DMatrixRMaj A, double t, int K) {
        int n = A.numRows;
        DMatrixRMaj I = CommonOps_DDRM.identity(n);

        DMatrixRMaj At = new DMatrixRMaj(n, n);
        CommonOps_DDRM.scale(t, A, At);

        DMatrixRMaj term = I.copy();   // (At^0)/0! = I
        DMatrixRMaj S = I.copy();

        DMatrixRMaj tmp = new DMatrixRMaj(n, n);

        for (int k = 1; k <= K; k++) {
            // term = term * (At / k)
            CommonOps_DDRM.mult(term, At, tmp);
            CommonOps_DDRM.scale(1.0 / k, tmp, term);
            CommonOps_DDRM.addEquals(S, term);
        }
        return S;
    }

    public static void main(String[] args) {
        // A = [[0, 1], [-2, -3]]
        DMatrixRMaj A = new DMatrixRMaj(new double[][]{
                {0.0, 1.0},
                {-2.0, -3.0}
        });

        double t = 1.0;
        DMatrixRMaj E = expmSeries(A, t, 50);

        System.out.println("||A t||_F = " + NormOps_DDRM.normF(A) * t);
        System.out.println("exp(A t) approx:");
        E.print();
    }
}
      

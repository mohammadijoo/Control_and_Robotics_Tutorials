import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class MatrixExponentialSeries {

    public static RealMatrix expmSeries(RealMatrix A, double t, double tol, int maxTerms) {
        int n = A.getRowDimension();
        RealMatrix S = MatrixUtils.createRealIdentityMatrix(n);
        RealMatrix term = MatrixUtils.createRealIdentityMatrix(n);
        RealMatrix At = A.scalarMultiply(t);

        for (int k = 1; k <= maxTerms; k++) {
            term = term.multiply(At).scalarMultiply(1.0 / (double) k);
            RealMatrix Snew = S.add(term);

            double termNormF = frobeniusNorm(term);
            double SNormF = frobeniusNorm(Snew);
            if (termNormF <= tol * SNormF) {
                return Snew;
            }
            S = Snew;
        }
        return S;
    }

    public static double frobeniusNorm(RealMatrix M) {
        double sum = 0.0;
        for (int i = 0; i < M.getRowDimension(); i++) {
            for (int j = 0; j < M.getColumnDimension(); j++) {
                double v = M.getEntry(i, j);
                sum += v * v;
            }
        }
        return Math.sqrt(sum);
    }

    public static void main(String[] args) {
        double[][] data = new double[][] {
            {0.0, 1.0},
            {-1.0, 0.0}
        };
        RealMatrix A = MatrixUtils.createRealMatrix(data);
        double t = 0.7;

        RealMatrix S = expmSeries(A, t, 1e-14, 400);
        System.out.println("Series exp(A t):");
        System.out.println(S.toString());
    }
}

import org.ejml.data.Complex_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;

public class RootLocusJava {

    public static DenseMatrix64F companion(double[] a) {
        int n = a.length;
        DenseMatrix64F C = new DenseMatrix64F(n, n);
        // Last row: negative coefficients
        for (int i = 0; i < n; ++i) {
            C.set(n - 1, i, -a[i]);
        }
        // Superdiagonal of ones
        for (int i = 0; i < n - 1; ++i) {
            C.set(i, i + 1, 1.0);
        }
        return C;
    }

    public static Complex_F64[] closedLoopPoles(double[] den, double[] num, double K) {
        int n = den.length - 1;
        double[] numPad = new double[den.length];
        int shift = den.length - num.length;
        for (int i = 0; i < num.length; ++i) {
            numPad[i + shift] = num[i];
        }
        double[] a = new double[n];
        for (int i = 0; i < n; ++i) {
            a[i] = den[i + 1] + K * numPad[i + 1];
        }
        DenseMatrix64F C = companion(a);
        EigenDecomposition<DenseMatrix64F> eig =
                DecompositionFactory.eig(n, false);
        eig.decompose(C);
        Complex_F64[] poles = new Complex_F64[n];
        for (int i = 0; i < n; ++i) {
            poles[i] = eig.getEigenvalue(i);
        }
        return poles;
    }

    public static void main(String[] args) {
        double[] num = {1.0, 1.0};          // s + 1
        double[] den = {1.0, 6.0, 8.0, 0.0}; // s^3 + 6 s^2 + 8 s

        double[] gains = {0.0, 10.0, 50.0, 200.0};
        for (double K : gains) {
            System.out.println("K = " + K);
            Complex_F64[] poles = closedLoopPoles(den, num, K);
            for (Complex_F64 p : poles) {
                System.out.println("  pole = " + p.real + " + j" + p.imaginary);
            }
        }
    }
}

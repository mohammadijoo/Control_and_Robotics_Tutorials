import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.QRDecomposition;

public class PendulumLeastSquares {

    public static RealMatrix buildRegressorPendulum(double[] q,
                                                    double[] qd,
                                                    double[] qdd) {
        int N = q.length;
        double[][] data = new double[N][3];
        for (int k = 0; k < N; ++k) {
            data[k][0] = qdd[k];
            data[k][1] = Math.sin(q[k]);
            data[k][2] = qd[k];
        }
        return MatrixUtils.createRealMatrix(data);
    }

    public static void main(String[] args) {
        // Assume q, qd, qdd, tauMeas are filled from logs
        double[] q, qd, qdd, tauMeas;
        // ... allocate and fill arrays ...

        int N = q.length;
        RealMatrix Y = buildRegressorPendulum(q, qd, qdd);

        double[] tData = new double[N];
        for (int k = 0; k < N; ++k) {
            tData[k] = tauMeas[k];
        }
        RealVector T = MatrixUtils.createRealVector(tData);

        // Solve Y * phi = T in least squares sense via QR decomposition
        QRDecomposition qr = new QRDecomposition(Y);
        DecompositionSolver solver = qr.getSolver();
        RealVector phiHat = solver.solve(T);

        System.out.println("Estimated parameters (a1, a2, a3): "
                           + phiHat);
    }
}
      

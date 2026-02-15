
import org.ejml.simple.SimpleMatrix;

public class SimpleGP1D {
    private SimpleMatrix X;       // N x d
    private SimpleMatrix y;       // N x 1
    private SimpleMatrix alpha;   // N x 1, Ky^{-1} y

    private double ell;
    private double sigmaF;
    private double sigmaN;
    private boolean trained = false;

    public SimpleGP1D(double ell, double sigmaF, double sigmaN) {
        this.ell = ell;
        this.sigmaF = sigmaF;
        this.sigmaN = sigmaN;
    }

    private double kernel(SimpleMatrix x1, SimpleMatrix x2) {
        SimpleMatrix diff = x1.minus(x2);
        double r2 = diff.transpose().mult(diff).get(0);
        return sigmaF * sigmaF * Math.exp(-0.5 * r2 / (ell * ell));
    }

    public void setTrainingData(double[][] Xdata, double[] ydata) {
        int N = ydata.length;
        int d = Xdata[0].length;
        X = new SimpleMatrix(N, d);
        y = new SimpleMatrix(N, 1);

        for (int i = 0; i < N; ++i) {
            y.set(i, 0, ydata[i]);
            for (int j = 0; j < d; ++j) {
                X.set(i, j, Xdata[i][j]);
            }
        }

        SimpleMatrix K = new SimpleMatrix(N, N);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                SimpleMatrix xi = X.extractVector(true, i);
                SimpleMatrix xj = X.extractVector(true, j);
                K.set(i, j, kernel(xi, xj));
            }
        }
        SimpleMatrix Ky = K.plus(SimpleMatrix.identity(N).scale(sigmaN * sigmaN));
        alpha = Ky.solve(y);
        trained = true;
    }

    public double predict(double[] xStar) {
        if (!trained) {
            throw new IllegalStateException("GP not trained");
        }
        int N = X.numRows();
        SimpleMatrix kStar = new SimpleMatrix(N, 1);
        SimpleMatrix xStarMat = new SimpleMatrix(1, xStar.length, true, xStar);
        for (int i = 0; i < N; ++i) {
            SimpleMatrix xi = X.extractVector(true, i);
            kStar.set(i, 0, kernel(xi, xStarMat));
        }
        double mean = kStar.transpose().mult(alpha).get(0);
        return mean;
    }
}

// Example usage inside a joint controller
public class JointController {
    private SimpleGP1D gp;
    // Nominal dynamics and PD gains are assumed to be implemented elsewhere.

    public JointController(SimpleGP1D gpModel) {
        this.gp = gpModel;
    }

    public double computeTorque(double q, double dq,
                                double qd, double dqd, double ddqd,
                                double tauNominal) {
        double[] x = new double[] { q, dq, ddqd };
        double dHat = gp.predict(x);
        return tauNominal + dHat;
    }
}

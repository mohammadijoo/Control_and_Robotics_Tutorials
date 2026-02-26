import org.ejml.simple.SimpleMatrix;
import java.util.Random;

public class PendulumUncertaintyJava {

    public static SimpleMatrix regressor(double q, double dq, double ddq) {
        double[] data = {ddq, dq, Math.sin(q)};
        return new SimpleMatrix(1, 3, true, data);
    }

    public static void main(String[] args) {
        double[] thetaHat = {0.05, 0.01, 0.5};
        double[] delta = {0.005, 0.002, 0.05};

        SimpleMatrix thetaHatMat =
            new SimpleMatrix(3, 1, true, thetaHat);

        double q = Math.toRadians(30.0);
        double dq = 0.5;
        double ddq = 1.0;

        SimpleMatrix Y = regressor(q, dq, ddq);
        double tauHat = Y.mult(thetaHatMat).get(0, 0);

        // Analytic interval bound
        double tauIntervalRadius =
            Math.abs(Y.get(0, 0)) * delta[0] +
            Math.abs(Y.get(0, 1)) * delta[1] +
            Math.abs(Y.get(0, 2)) * delta[2];

        System.out.println("Nominal tau: " + tauHat);
        System.out.println("Interval bound |tau - tauHat|: " + tauIntervalRadius);

        // Monte Carlo check
        Random rng = new Random(0);
        int N = 10000;
        double maxErr = 0.0;
        for (int k = 0; k < N; ++k) {
            double[] thetaSample = new double[3];
            for (int i = 0; i < 3; ++i) {
                double u = rng.nextDouble(); // in [0,1]
                thetaSample[i] = thetaHat[i] + (2.0 * u - 1.0) * delta[i];
            }
            SimpleMatrix thetaSampleMat =
                new SimpleMatrix(3, 1, true, thetaSample);
            double tauSample = Y.mult(thetaSampleMat).get(0, 0);
            double err = Math.abs(tauSample - tauHat);
            if (err > maxErr) {
                maxErr = err;
            }
        }
        System.out.println("Max error over samples: " + maxErr);
    }
}
      

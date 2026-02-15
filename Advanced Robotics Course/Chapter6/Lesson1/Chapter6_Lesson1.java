import java.util.Random;

public class StochasticUnicycle {

    static final double DT = 0.1;
    static final Random rng = new Random(42L);

    // x = [x, y, phi]
    static double[] f(double[] x, double[] u) {
        double px = x[0];
        double py = x[1];
        double phi = x[2];
        double v = u[0];
        double omega = u[1];

        return new double[] {
            px + v * Math.cos(phi) * DT,
            py + v * Math.sin(phi) * DT,
            phi + omega * DT
        };
    }

    static double[] sampleProcessNoise(double sigmaX, double sigmaY, double sigmaPhi) {
        return new double[] {
            sigmaX * rng.nextGaussian(),
            sigmaY * rng.nextGaussian(),
            sigmaPhi * rng.nextGaussian()
        };
    }

    public static void main(String[] args) {
        double[] x = new double[] {0.0, 0.0, 0.0};
        double[] u = new double[] {1.0, 0.2};

        // Standard deviations of process noise
        double sigmaX = 0.01;
        double sigmaY = 0.01;
        double sigmaPhi = Math.toRadians(1.0);

        for (int t = 0; t < 20; ++t) {
            double[] xNom = f(x, u);
            double[] w = sampleProcessNoise(sigmaX, sigmaY, sigmaPhi);
            for (int i = 0; i < 3; ++i) {
                x[i] = xNom[i] + w[i];
            }
        }

        System.out.printf("Final stochastic state: x=%.3f, y=%.3f, phi=%.3f%n",
                          x[0], x[1], x[2]);
    }
}
      

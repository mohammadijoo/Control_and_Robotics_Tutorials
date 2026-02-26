import java.util.Random;

public class ProbabilisticDynamics1DOF {

    // Regressor Y(q, qd, qdd) = [qdd, sin(q)]
    static double[] regressor(double q, double qd, double qdd) {
        return new double[]{qdd, Math.sin(q)};
    }

    public static void main(String[] args) {
        // Parameter mean and (diagonal) covariance
        double[] thetaMean = {1.5, 0.8};
        double[] thetaStd  = {0.2, 0.1}; // sqrt of variances

        double q   = 0.5;
        double qd  = 0.0;
        double qdd = 2.0;
        double[] Y = regressor(q, qd, qdd); // length 2

        // Analytical mean and variance (independent parameters)
        double tauMean = Y[0] * thetaMean[0] + Y[1] * thetaMean[1];
        double tauVar  =
                Y[0] * Y[0] * thetaStd[0] * thetaStd[0]
              + Y[1] * Y[1] * thetaStd[1] * thetaStd[1];

        System.out.println("Analytical E[tau] = " + tauMean);
        System.out.println("Analytical Var[tau] = " + tauVar);

        // Monte Carlo sampling
        Random rng = new Random(0L);
        int Nmc = 100000;
        double meanMc = 0.0;
        double m2Mc   = 0.0;

        for (int k = 0; k < Nmc; ++k) {
            double aSample = thetaMean[0] + thetaStd[0] * rng.nextGaussian();
            double bSample = thetaMean[1] + thetaStd[1] * rng.nextGaussian();

            double tauSample = Y[0] * aSample + Y[1] * bSample;

            double delta = tauSample - meanMc;
            meanMc += delta / (k + 1);
            m2Mc   += delta * (tauSample - meanMc);
        }

        double varMc = m2Mc / (Nmc - 1);
        System.out.println("MC mean approx = " + meanMc);
        System.out.println("MC var approx  = " + varMc);
    }
}
      

// Chapter6_Lesson4.java
// Independence Assumptions and Their Limits — correlated-sensor Bayesian fusion (Java)
//
// Demonstrates overconfidence when correlation is ignored.
//
// Libraries: none required (pure Java). For larger-scale robotics estimation,
// you would typically integrate with rosjava or Apache Commons Math.
// Compile:
//   javac Chapter6_Lesson4.java
// Run:
//   java Chapter6_Lesson4

public class Chapter6_Lesson4 {

    static class Posterior {
        public double mean;
        public double var;
        Posterior(double mean, double var) { this.mean = mean; this.var = var; }
    }

    // Inverse of Sigma = sigma^2 [[1,rho],[rho,1]]
    static double[][] invSigma(double sigma, double rho) {
        double s2 = sigma * sigma;
        double det = s2 * s2 * (1.0 - rho * rho);
        double[][] inv = new double[2][2];
        inv[0][0] =  s2 / det;   // 1/(sigma^2*(1-rho^2))
        inv[0][1] = -rho*s2 / det;
        inv[1][0] = -rho*s2 / det;
        inv[1][1] =  s2 / det;
        return inv;
    }

    static Posterior posteriorCorrelated(double mu0, double sigma0, double y1, double y2, double sigma, double rho) {
        double[][] Sinv = invSigma(sigma, rho);

        // H = [1,1]^T. info_meas = H^T Sinv H = sum of all entries of Sinv
        double info_meas = Sinv[0][0] + Sinv[0][1] + Sinv[1][0] + Sinv[1][1];

        double postVar = 1.0 / (1.0/(sigma0*sigma0) + info_meas);

        // info_vec = H^T Sinv y
        double info_vec = (Sinv[0][0] + Sinv[1][0]) * y1 + (Sinv[0][1] + Sinv[1][1]) * y2;
        double postMean = postVar * (mu0/(sigma0*sigma0) + info_vec);

        return new Posterior(postMean, postVar);
    }

    static Posterior posteriorIndependent(double mu0, double sigma0, double y1, double y2, double sigma) {
        double postVar = 1.0 / (1.0/(sigma0*sigma0) + 2.0/(sigma*sigma));
        double postMean = postVar * (mu0/(sigma0*sigma0) + (y1 + y2)/(sigma*sigma));
        return new Posterior(postMean, postVar);
    }

    public static void main(String[] args) {
        double mu0 = 0.0;
        double sigma0 = 2.0;
        double sigma = 1.0;
        double y1 = 1.0, y2 = 1.2;

        System.out.println(String.format("Prior: x ~ N(mu0, sigma0^2), mu0=%.6f, sigma0=%.6f", mu0, sigma0));
        System.out.println(String.format("Measurements: y1=%.6f, y2=%.6f, sigma=%.6f\n", y1, y2, sigma));

        double[] rhos = new double[] {0.0, 0.3, 0.6, 0.9};
        for (double rho : rhos) {
            Posterior pc = posteriorCorrelated(mu0, sigma0, y1, y2, sigma, rho);
            Posterior pi = posteriorIndependent(mu0, sigma0, y1, y2, sigma);
            double ratio = pi.var / pc.var;

            System.out.println(String.format("rho=%.1f:", rho));
            System.out.println(String.format("  Correct correlated posterior: mean=%.6f, var=%.6f", pc.mean, pc.var));
            System.out.println(String.format("  Indep. assumption posterior:  mean=%.6f, var=%.6f", pi.mean, pi.var));
            System.out.println(String.format("  Variance ratio (indep/correct) = %.6f ( < 1 means overconfident )\n", ratio));
        }
    }
}

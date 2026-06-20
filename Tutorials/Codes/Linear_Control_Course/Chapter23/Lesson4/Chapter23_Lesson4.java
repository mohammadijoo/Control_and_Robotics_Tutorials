public class MultiplicativeUncertaintyDemo {

    // Magnitude of G(jw) = K / (1 + j w tau)
    static double magG(double w, double K, double tau) {
        double denomRe = 1.0;
        double denomIm = w * tau;
        double denomMag = Math.sqrt(denomRe * denomRe + denomIm * denomIm);
        return K / denomMag;
    }

    public static void main(String[] args) {
        double K0 = 10.0;
        double tau0 = 0.05;
        double alpha = 0.2;
        double beta  = 0.1;

        double[] w = new double[50];
        for (int i = 0; i < w.length; ++i) {
            // logspace from 1 to 1000 rad/s
            double t = (double) i / (w.length - 1);
            w[i] = Math.pow(10.0, 0.0 + 3.0 * t);
        }

        // Example uncertain parameters at extremes
        double K_unc = K0 * (1.0 + alpha);
        double tau_unc = tau0 * (1.0 - beta);

        System.out.println("w, |Delta_M(jw)|");
        for (int i = 0; i < w.length; ++i) {
            double wi = w[i];
            double G0mag = magG(wi, K0, tau0);
            double Gmag  = magG(wi, K_unc, tau_unc);
            double deltaM = Math.abs(Gmag - G0mag) / G0mag;
            System.out.println(wi + ", " + deltaM);
        }
    }
}

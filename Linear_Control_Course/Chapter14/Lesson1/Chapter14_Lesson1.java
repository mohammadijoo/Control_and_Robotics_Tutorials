public class DecibelUtils {

    // Convert amplitude ratio to dB
    public static double ampToDb(double amp) {
        return 20.0 * Math.log10(amp);
    }

    // Magnitude of first-order plant G(s) = K / (tau*s + 1) at frequency w
    public static double firstOrderMag(double K, double tau, double w) {
        // jw = j*w, so |1 + tau*jw| = sqrt(1 + (tau*w)^2)
        double denomMag = Math.sqrt(1.0 + Math.pow(tau * w, 2));
        return K / denomMag;
    }

    public static void main(String[] args) {
        double K = 10.0;
        double tau = 0.05;
        double[] wValues = {0.1, 1.0, 10.0, 100.0};

        for (double w : wValues) {
            double mag = firstOrderMag(K, tau, w);
            double db  = ampToDb(mag);
            System.out.printf("w = %.2f rad/s, |G(jw)| = %.4f, M_dB = %.2f dB%n",
                              w, mag, db);
        }
    }
}

public class SensitivityWaterbed {
    public static double S_mag(double w, double k) {
        double num = w * w + 1.0;
        double a = 1.0 + k - w * w;
        double b = 2.0 * w;
        double den = Math.sqrt(a * a + b * b);
        return num / den;
    }

    public static void main(String[] args) {
        double k = 5.0;
        double[] w = {0.1, 0.3, 1.0, 3.0, 10.0, 30.0};

        for (int i = 0; i != w.length; ++i) {
            double wi = w[i];
            double mag = S_mag(wi, k);
            double magDb = 20.0 * Math.log10(mag);
            System.out.println("w = " + wi + " rad/s, |S(jw)| = "
                               + mag + " (" + magDb + " dB)");
        }
    }
}

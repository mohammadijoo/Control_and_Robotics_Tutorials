// Example: evaluate PID loop frequency response for a first-order plant
// and print approximate crossover frequency and phase margin.
//
// For robotics, the tuned gains can be used with
//   import edu.wpi.first.math.controller.PIDController;

public class PIDFrequencyDesigner {

    // Plant G(s) = K / (T s + 1)
    static double[] plantReIm(double K, double T, double w) {
        double denom = 1.0 + (w * T) * (w * T);
        double re = K / denom;
        double im = -K * w * T / denom;
        return new double[]{re, im};
    }

    // PID C(s) = Kp + Ki/s + Kd s at s = j w
    static double[] pidReIm(double Kp, double Ki, double Kd, double w) {
        double re = Kp;
        double im = Kd * w - Ki / w;
        return new double[]{re, im};
    }

    static double mag(double re, double im) {
        return Math.sqrt(re * re + im * im);
    }

    static double phase(double re, double im) {
        return Math.atan2(im, re); // radians
    }

    public static void main(String[] args) {
        double K = 20.0;
        double T = 0.1;

        // Gains from design (for example, derived analytically or using Python)
        double Kp = 1.1;
        double Ki = 11.0;
        double Kd = 0.02;

        // Sweep frequency and approximate crossover
        double omegaC = 0.0;
        double phaseAtC = 0.0;
        double prevMag = Double.POSITIVE_INFINITY;
        for (double w = 1.0; w <= 200.0; w += 1.0) {
            double[] g = plantReIm(K, T, w);
            double[] c = pidReIm(Kp, Ki, Kd, w);

            double reL = g[0] * c[0] - g[1] * c[1];
            double imL = g[0] * c[1] + g[1] * c[0];
            double magL = mag(reL, imL);

            if (magL <= 1.0 && prevMag > 1.0) {
                omegaC = w;
                phaseAtC = phase(reL, imL);
                break;
            }
            prevMag = magL;
        }

        double pm = Math.PI + phaseAtC; // radians
        System.out.println("Approx crossover omega_c = " + omegaC + " rad/s");
        System.out.println("Approx phase margin = " + pm * 180.0 / Math.PI + " deg");

        // Use these gains in a WPILib controller:
        // PIDController controller = new PIDController(Kp, Ki, Kd);
    }
}

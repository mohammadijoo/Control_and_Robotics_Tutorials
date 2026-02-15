import org.apache.commons.math3.complex.Complex;
import java.util.Random;

// Simple discrete-time closed loop: x_{k+1} = a x_k + b u_k, y_k = x_k
// with u_k = -Kc (y_k - r_k), here r_k = 1 (step input).
public class MonteCarloRobustness {

    static double simulateClosedLoop(double a, double b, double Kc, int steps) {
        double x = 0.0;
        double r = 1.0;
        double maxOvershoot = 0.0;
        for (int k = 0; k < steps; ++k) {
            double y = x;
            double e = r - y;
            double u = Kc * e;
            x = a * x + b * u;
            if (y > maxOvershoot) {
                maxOvershoot = y;
            }
        }
        return maxOvershoot;
    }

    public static void main(String[] args) {
        double Ts = 0.001;   // sample time
        double J_nom = 0.01;
        double B_nom = 0.1;
        double K_gain = 1.0;
        double Kc = 2.0;

        // First-order plant discretization (Euler) for nominal parameters:
        // dx/dt = -(B/J) x + (K/J) u
        // a = 1 + Ts * (-(B/J)), b = Ts * (K/J)
        Random rng = new Random(0);
        int trials = 100;
        int unstableCount = 0;

        for (int i = 0; i < trials; ++i) {
            // Randomize J and B by ±40 %
            double J = J_nom * (0.6 + 0.8 * rng.nextDouble());
            double B = B_nom * (0.6 + 0.8 * rng.nextDouble());

            double a = 1.0 + Ts * (-(B / J));
            double b = Ts * (K_gain / J);

            double overshoot = simulateClosedLoop(a, b, Kc, 10000);

            // A crude stability/robustness check:
            // if overshoot is extremely large, we treat it as "unstable/fragile".
            if (Double.isNaN(overshoot) || overshoot > 10.0) {
                unstableCount++;
            }
        }

        System.out.println("Trials with large overshoot or numerical blow-up: "
                + unstableCount + " out of " + trials);
    }
}

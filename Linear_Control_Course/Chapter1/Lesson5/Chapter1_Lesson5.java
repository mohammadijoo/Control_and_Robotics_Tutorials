import java.util.function.BiFunction;

public class LinearityTest {

    // System interface: y[i] = S(u[i], t[i])
    @FunctionalInterface
    public interface DiscreteSystem {
        double[] apply(double[] u, double[] t);
    }

    public static double[] stepInput(double[] t) {
        double[] u = new double[t.length];
        for (int i = 0; i < t.length; i++) {
            u[i] = (t[i] >= 0.0) ? 1.0 : 0.0;
        }
        return u;
    }

    public static double[] rampInput(double[] t) {
        double[] u = new double[t.length];
        for (int i = 0; i < t.length; i++) {
            u[i] = t[i];
        }
        return u;
    }

    public static double linearityError(DiscreteSystem S, double[] t) {
        double alpha = 1.5;
        double beta = -0.4;

        double[] u1 = stepInput(t);
        double[] u2 = rampInput(t);
        double[] uCombo = new double[t.length];

        for (int i = 0; i < t.length; i++) {
            uCombo[i] = alpha * u1[i] + beta * u2[i];
        }

        double[] yCombo = S.apply(uCombo, t);
        double[] yLin1 = S.apply(u1, t);
        double[] yLin2 = S.apply(u2, t);

        double err2 = 0.0;
        for (int i = 0; i < t.length; i++) {
            double yLin = alpha * yLin1[i] + beta * yLin2[i];
            double diff = yCombo[i] - yLin;
            err2 += diff * diff;
        }
        return Math.sqrt(err2);
    }

    public static void main(String[] args) {
        int N = 501;
        double[] t = new double[N];
        double t0 = 0.0;
        double tf = 5.0;
        double dt = (tf - t0) / (N - 1);
        for (int i = 0; i < N; i++) {
            t[i] = t0 + i * dt;
        }

        // LTI gain system
        DiscreteSystem S1 = (u, time) -> {
            double k = 2.0;
            double[] y = new double[u.length];
            for (int i = 0; i < u.length; i++) {
                y[i] = k * u[i];
            }
            return y;
        };

        // Nonlinear squaring system
        DiscreteSystem S3 = (u, time) -> {
            double[] y = new double[u.length];
            for (int i = 0; i < u.length; i++) {
                y[i] = u[i] * u[i];
            }
            return y;
        };

        System.out.println("S1 linearity error = " + linearityError(S1, t));
        System.out.println("S3 linearity error = " + linearityError(S3, t));
    }
}

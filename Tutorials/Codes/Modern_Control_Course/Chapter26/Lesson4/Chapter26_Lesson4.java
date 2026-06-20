/*
Chapter26_Lesson4.java
From-scratch RK4 simulation for step and ramp tracking with integral action.

Compile:
    javac Chapter26_Lesson4.java
Run:
    java Chapter26_Lesson4
*/

public class Chapter26_Lesson4 {
    static double[] add(double[] a, double[] b, double scale) {
        double[] c = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            c[i] = a[i] + scale * b[i];
        }
        return c;
    }

    static double[] stepDynamics(double t, double[] z) {
        double r = 1.0;
        // K_step = [72, 12, -120], u = -K_step z.
        double u = -72.0 * z[0] - 12.0 * z[1] + 120.0 * z[2];
        double x1dot = z[1];
        double x2dot = -2.0 * z[0] - 3.0 * z[1] + u;
        double etadot = r - z[0];
        return new double[] {x1dot, x2dot, etadot};
    }

    static double[] rampDynamics(double t, double[] z) {
        double r = t;
        // K_ramp = [117, 15, -342, -360], u = -K_ramp z.
        double u = -117.0 * z[0] - 15.0 * z[1] + 342.0 * z[2] + 360.0 * z[3];
        double x1dot = z[1];
        double x2dot = -2.0 * z[0] - 3.0 * z[1] + u;
        double eta1dot = r - z[0];
        double eta2dot = z[2];
        return new double[] {x1dot, x2dot, eta1dot, eta2dot};
    }

    static double[] rk4(double t, double[] z, double h, boolean ramp) {
        double[] k1 = ramp ? rampDynamics(t, z) : stepDynamics(t, z);
        double[] k2 = ramp ? rampDynamics(t + h / 2.0, add(z, k1, h / 2.0))
                           : stepDynamics(t + h / 2.0, add(z, k1, h / 2.0));
        double[] k3 = ramp ? rampDynamics(t + h / 2.0, add(z, k2, h / 2.0))
                           : stepDynamics(t + h / 2.0, add(z, k2, h / 2.0));
        double[] k4 = ramp ? rampDynamics(t + h, add(z, k3, h))
                           : stepDynamics(t + h, add(z, k3, h));
        double[] out = new double[z.length];
        for (int i = 0; i < z.length; i++) {
            out[i] = z[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        }
        return out;
    }

    public static void main(String[] args) {
        double h = 0.001;
        double tf = 8.0;
        int steps = (int) (tf / h);

        double[] zStep = {0.0, 0.0, 0.0};
        double[] zRamp = {0.0, 0.0, 0.0, 0.0};

        for (int k = 0; k < steps; k++) {
            double t = k * h;
            zStep = rk4(t, zStep, h, false);
            zRamp = rk4(t, zRamp, h, true);
        }

        double stepError = 1.0 - zStep[0];
        double rampError = tf - zRamp[0];

        System.out.printf("Final step output y(tf) = %.10f%n", zStep[0]);
        System.out.printf("Final step error        = %.10f%n", stepError);
        System.out.printf("Final ramp output y(tf) = %.10f%n", zRamp[0]);
        System.out.printf("Final ramp error        = %.10f%n", rampError);
    }
}

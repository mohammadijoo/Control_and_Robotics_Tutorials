// Chapter28_Lesson2.java
// Energy-like measures for state-space signals using only the Java standard library.
// Java libraries useful for modern control/numerics include EJML, Apache Commons Math, ojAlgo, and ND4J.

public class Chapter28_Lesson2 {
    static double input(double t) {
        return 0.5 * Math.sin(3.0 * t) * Math.exp(-0.2 * t);
    }

    static double[] dynamics(double t, double[] x) {
        double u = input(t);
        return new double[] {x[1], -4.0 * x[0] - 0.8 * x[1] + u};
    }

    static double[] addScaled(double[] x, double[] k, double h) {
        return new double[] {x[0] + h * k[0], x[1] + h * k[1]};
    }

    static double[] rk4Step(double t, double[] x, double h) {
        double[] k1 = dynamics(t, x);
        double[] k2 = dynamics(t + 0.5 * h, addScaled(x, k1, 0.5 * h));
        double[] k3 = dynamics(t + 0.5 * h, addScaled(x, k2, 0.5 * h));
        double[] k4 = dynamics(t + h, addScaled(x, k3, h));
        return new double[] {
            x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        };
    }

    static double stateQuadratic(double[] x) {
        return 10.0 * x[0] * x[0] + x[1] * x[1];
    }

    static double outputWeightedQuadratic(double[] y) {
        return y[0] * y[0] + 0.1 * y[1] * y[1];
    }

    static double norm2(double[] y) {
        return Math.sqrt(y[0] * y[0] + y[1] * y[1]);
    }

    public static void main(String[] args) {
        double t0 = 0.0;
        double tf = 12.0;
        double h = 0.01;
        int n = (int) Math.round((tf - t0) / h);

        double[] t = new double[n + 1];
        double[][] x = new double[n + 1][2];
        double[] u = new double[n + 1];

        x[0][0] = 1.0;
        x[0][1] = 0.0;
        t[0] = t0;
        u[0] = input(t0);

        for (int i = 0; i < n; i++) {
            t[i + 1] = t[i] + h;
            x[i + 1] = rk4Step(t[i], x[i], h);
            u[i + 1] = input(t[i + 1]);
        }

        double stateEnergy = 0.0;
        double inputEnergy = 0.0;
        double outputEnergy = 0.0;
        double linf = 0.0;

        for (int i = 0; i < n; i++) {
            double dt = t[i + 1] - t[i];
            stateEnergy += 0.5 * dt * (stateQuadratic(x[i]) + stateQuadratic(x[i + 1]));
            inputEnergy += 0.5 * dt * (0.2 * u[i] * u[i] + 0.2 * u[i + 1] * u[i + 1]);
            outputEnergy += 0.5 * dt * (outputWeightedQuadratic(x[i]) + outputWeightedQuadratic(x[i + 1]));
            linf = Math.max(linf, norm2(x[i]));
        }

        System.out.println("Weighted state energy   = " + stateEnergy);
        System.out.println("Weighted input energy   = " + inputEnergy);
        System.out.println("Performance J           = " + (stateEnergy + inputEnergy));
        System.out.println("Weighted output L2 norm = " + Math.sqrt(outputEnergy));
        System.out.println("Weighted output RMS     = " + Math.sqrt(outputEnergy / (tf - t0)));
        System.out.println("Output L-infinity norm  = " + linf);
    }
}

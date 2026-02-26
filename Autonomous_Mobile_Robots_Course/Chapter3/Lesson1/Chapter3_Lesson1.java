/*
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 1 — Nonholonomic Constraints in Wheeled Robots (applied view)

Differential-drive kinematic simulation and lateral constraint residual check.

Build:
  javac Chapter3_Lesson1.java
Run:
  java Chapter3_Lesson1

Robotics-adjacent libraries you may integrate in larger projects:
  - EJML (linear algebra), JOML (geometry), ROSJava (middleware bindings)
  - This example uses only core Java for portability.
*/

public class Chapter3_Lesson1 {

    static class SimResult {
        double[] t, x, y, th, v, w, res;
    }

    static void diffDriveTwist(double omegaL, double omegaR, double r, double b, double[] outVW) {
        double v = 0.5 * r * (omegaR + omegaL);
        double w = 0.5 * r * (omegaR - omegaL) / b;
        outVW[0] = v;
        outVW[1] = w;
    }

    static double lateralConstraintResidual(double xDot, double yDot, double theta) {
        // -sin(theta) x_dot + cos(theta) y_dot = 0
        return -Math.sin(theta) * xDot + Math.cos(theta) * yDot;
    }

    static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }

    static void unicycleStep(double[] pose, double v, double w, double dt) {
        double x = pose[0], y = pose[1], th = pose[2];
        x += dt * v * Math.cos(th);
        y += dt * v * Math.sin(th);
        th = wrapToPi(th + dt * w);
        pose[0] = x; pose[1] = y; pose[2] = th;
    }

    static void wheelProfile(double t, double[] outOmegaLR) {
        if (t < 4.0) {
            outOmegaLR[0] = 6.0; outOmegaLR[1] = 6.0;
        } else if (t < 8.0) {
            outOmegaLR[0] = 3.0; outOmegaLR[1] = 7.0;
        } else if (t < 12.0) {
            outOmegaLR[0] = 7.0; outOmegaLR[1] = 3.0;
        } else {
            outOmegaLR[0] = 5.0; outOmegaLR[1] = 5.0;
        }
    }

    static SimResult runSim(double T, double dt, double r, double b) {
        int N = (int)Math.floor(T / dt) + 1;

        SimResult s = new SimResult();
        s.t = new double[N];
        s.x = new double[N];
        s.y = new double[N];
        s.th = new double[N];
        s.v = new double[N];
        s.w = new double[N];
        s.res = new double[N];

        double[] pose = new double[] {0.0, 0.0, 0.0};
        double[] omegas = new double[2];
        double[] vw = new double[2];

        for (int k = 0; k < N; ++k) {
            double tk = k * dt;
            s.t[k] = tk;

            wheelProfile(tk, omegas);
            diffDriveTwist(omegas[0], omegas[1], r, b, vw);

            double v = vw[0], w = vw[1];
            double xDot = v * Math.cos(pose[2]);
            double yDot = v * Math.sin(pose[2]);

            s.x[k] = pose[0];
            s.y[k] = pose[1];
            s.th[k] = pose[2];
            s.v[k] = v;
            s.w[k] = w;
            s.res[k] = lateralConstraintResidual(xDot, yDot, pose[2]);

            if (k < N - 1) {
                unicycleStep(pose, v, w, dt);
            }
        }
        return s;
    }

    public static void main(String[] args) {
        SimResult sim = runSim(16.0, 0.01, 0.10, 0.22);

        double maxAbs = 0.0;
        for (double e : sim.res) maxAbs = Math.max(maxAbs, Math.abs(e));

        System.out.println("max |constraint residual| = " + maxAbs);
        System.out.println("final pose: x=" + sim.x[sim.x.length - 1]
                + " y=" + sim.y[sim.y.length - 1]
                + " theta=" + sim.th[sim.th.length - 1]);

        // For plotting, export to CSV and use your preferred plotting tool (exercise).
    }
}

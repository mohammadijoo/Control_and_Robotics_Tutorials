// Chapter19_Lesson4.java
// System Dynamics — Chapter 19, Lesson 4
// Discrete-time delay line (ring buffer) + N-lag chain approximation.

import java.util.Locale;

class DelayLine {
    private final double[] buf;
    private int idx;

    public DelayLine(int delaySamples) {
        int n = Math.max(1, delaySamples + 1);
        buf = new double[n];
        idx = 0;
    }

    public double step(double u) {
        double y = buf[idx];
        buf[idx] = u;
        idx = (idx + 1) % buf.length;
        return y;
    }
}

class LagChain {
    private final double T;
    private final int N;
    private final double[] x;

    public LagChain(double T, int N) {
        if (T <= 0.0) throw new IllegalArgumentException("T must be positive.");
        if (N < 1) throw new IllegalArgumentException("N must be >= 1.");
        this.T = T;
        this.N = N;
        this.x = new double[N];
    }

    // Euler update of the continuous-time chain:
    // x1dot = (N/T)(u - x1)
    // xkdot = (N/T)(x_{k-1} - xk)
    public double step(double u, double dt) {
        double alpha = ((double) N) / T;
        double[] xnew = x.clone();
        xnew[0] = x[0] + dt * alpha * (u - x[0]);
        for (int k = 1; k < N; k++) {
            xnew[k] = x[k] + dt * alpha * (x[k - 1] - x[k]);
        }
        System.arraycopy(xnew, 0, x, 0, N);
        return x[N - 1];
    }
}

public class Chapter19_Lesson4 {
    public static void main(String[] args) {
        Locale.setDefault(Locale.US);

        double dt = 0.001;
        double tEnd = 8.0;

        // Plant: ydot = -a y + b u_del
        double a = 1.0, b = 1.0;

        // Delay settings
        double T = 1.0;
        int delaySamples = (int) Math.round(T / dt);

        DelayLine delay = new DelayLine(delaySamples);
        LagChain chain = new LagChain(T, 10);

        double yDelay = 0.0;
        double yChain = 0.0;

        System.out.println("# t, y_trueDelay, y_chainApprox");
        int steps = (int) Math.round(tEnd / dt);

        for (int k = 0; k <= steps; k++) {
            double t = k * dt;
            double u = (t >= 0.0) ? 1.0 : 0.0;

            double uDel = delay.step(u);
            double uChain = chain.step(u, dt);

            yDelay = yDelay + dt * (-a * yDelay + b * uDel);
            yChain = yChain + dt * (-a * yChain + b * uChain);

            if (k % 50 == 0) {
                System.out.printf("%.6f, %.6f, %.6f%n", t, yDelay, yChain);
            }
        }

        System.out.println("\n# The delay line is an exact integer-sample delay.");
        System.out.println("# The chain is a stable approximation of a transport delay.");
    }
}

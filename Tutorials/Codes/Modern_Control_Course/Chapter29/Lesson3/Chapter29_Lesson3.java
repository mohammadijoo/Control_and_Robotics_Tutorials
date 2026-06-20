/*
Chapter29_Lesson3.java

Numerical experiments for Lesson 3:
Stability notions for continuous-time linear time-varying systems.

Build:
    javac Chapter29_Lesson3.java

Run:
    java Chapter29_Lesson3
*/

public class Chapter29_Lesson3 {
    static double[][] zeros() {
        return new double[2][2];
    }

    static double[][] identity() {
        double[][] M = zeros();
        M[0][0] = 1.0;
        M[1][1] = 1.0;
        return M;
    }

    static double[][] add(double[][] A, double[][] B) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    static double[][] scale(double[][] A, double s) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                C[i][j] = s * A[i][j];
            }
        }
        return C;
    }

    static double[][] multiply(double[][] A, double[][] B) {
        double[][] C = zeros();
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    static double frobeniusNorm(double[][] A) {
        double s = 0.0;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                s += A[i][j] * A[i][j];
            }
        }
        return Math.sqrt(s);
    }

    static double[][] aOfT(double t) {
        double[][] A = zeros();
        double k = 1.0 + 0.20 * Math.sin(t);
        double c = 0.80 + 0.10 * Math.cos(2.0 * t);
        A[0][0] = 0.0;
        A[0][1] = 1.0;
        A[1][0] = -k;
        A[1][1] = -c;
        return A;
    }

    static double[][] f(double t, double[][] Phi) {
        return multiply(aOfT(t), Phi);
    }

    static double[][] rk4Phi(double t0, double t1, double h) {
        if (t1 < t0) {
            throw new IllegalArgumentException("This routine assumes t1 >= t0.");
        }

        int n = (int)Math.ceil((t1 - t0) / h);
        if (n == 0) {
            return identity();
        }

        double hEff = (t1 - t0) / n;
        double t = t0;
        double[][] Phi = identity();

        for (int step = 0; step < n; step++) {
            double[][] K1 = f(t, Phi);
            double[][] K2 = f(t + 0.5 * hEff, add(Phi, scale(K1, 0.5 * hEff)));
            double[][] K3 = f(t + 0.5 * hEff, add(Phi, scale(K2, 0.5 * hEff)));
            double[][] K4 = f(t + hEff, add(Phi, scale(K3, hEff)));

            double[][] incr = add(add(K1, scale(K2, 2.0)), add(scale(K3, 2.0), K4));
            Phi = add(Phi, scale(incr, hEff / 6.0));
            t += hEff;
        }

        return Phi;
    }

    static double phiUniformExponentialScalar(double t, double t0) {
        return Math.exp(-0.5 * (t - t0) + 0.25 * (Math.cos(t0) - Math.cos(t)));
    }

    static double phiUniformStableNotUniformAttractive(double t, double t0) {
        return (1.0 + t0) / (1.0 + t);
    }

    public static void main(String[] args) {
        System.out.println("Scalar uniformly exponentially stable example");
        double M = Math.exp(0.5);
        double alpha = 0.5;
        double[] taus = new double[] {0.0, 1.0, 2.0, 5.0, 10.0};

        for (double tau : taus) {
            double maxRatio = 0.0;
            for (int i = 0; i <= 40; i++) {
                double t0 = 0.5 * i;
                double phi = Math.abs(phiUniformExponentialScalar(t0 + tau, t0));
                double bound = M * Math.exp(-alpha * tau);
                maxRatio = Math.max(maxRatio, phi / bound);
            }
            System.out.printf("tau=%.1f, max |Phi|/bound = %.6f%n", tau, maxRatio);
        }

        System.out.println();
        System.out.println("Uniformly stable but not uniformly attractive example");
        double[] Ts = new double[] {1.0, 5.0, 20.0};
        double[] t0s = new double[] {0.0, 10.0, 100.0, 1000.0, 10000.0};

        for (double T : Ts) {
            System.out.printf("T=%.1f: ", T);
            for (double t0 : t0s) {
                System.out.printf("%.6f ", phiUniformStableNotUniformAttractive(t0 + T, t0));
            }
            System.out.println();
        }

        System.out.println();
        System.out.println("2x2 transition-matrix Frobenius norm estimates");
        for (double tau : new double[] {1.0, 2.0, 5.0, 10.0}) {
            double maxNorm = 0.0;
            for (int i = 0; i <= 5; i++) {
                double t0 = (double)i;
                double[][] Phi = rk4Phi(t0, t0 + tau, 0.005);
                maxNorm = Math.max(maxNorm, frobeniusNorm(Phi));
            }
            System.out.printf("tau=%.1f, max sampled ||Phi||_F = %.6f%n", tau, maxNorm);
        }
    }
}

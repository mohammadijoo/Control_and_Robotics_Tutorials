// Chapter18_Lesson3.java
// Long-Range Localization Drift Handling (2D EKF, pure Java implementation)
// State: [x, y, psi, b_g]^T
// Compile: javac Chapter18_Lesson3.java
// Run:     java Chapter18_Lesson3

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter18_Lesson3 {
    static final double PI = Math.PI;

    static double wrapAngle(double a) {
        while (a > PI) a -= 2.0 * PI;
        while (a < -PI) a += 2.0 * PI;
        return a;
    }

    static class FilterState {
        double[] x = new double[4];
        double[][] P = new double[4][4];
    }

    static double[][] eye4() {
        double[][] I = new double[4][4];
        for (int i = 0; i < 4; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] matMul(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                for (int j = 0; j < 4; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] matT(double[][] A) {
        double[][] T = new double[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = A[j][i];
        return T;
    }

    static void symmetrize(double[][] P) {
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                double s = 0.5 * (P[i][j] + P[j][i]);
                P[i][j] = s;
                P[j][i] = s;
            }
        }
    }

    static void propagate(FilterState s, double vm, double wm, double dt, boolean rough) {
        double psi = s.x[2];
        double bg = s.x[3];

        s.x[0] += vm * dt * Math.cos(psi);
        s.x[1] += vm * dt * Math.sin(psi);
        s.x[2] = wrapAngle(s.x[2] + (wm - bg) * dt);

        double[][] F = eye4();
        F[0][2] = -vm * dt * Math.sin(psi);
        F[1][2] =  vm * dt * Math.cos(psi);
        F[2][3] = -dt;

        double sigmaV = rough ? 0.12 : 0.03;
        double sigmaW = rough ? 0.06 : 0.01;
        double sigmaBg = rough ? 0.0030 : 0.0008;

        double[][] Q = new double[4][4];
        Q[0][0] = Math.pow(sigmaV * dt, 2);
        Q[1][1] = Math.pow(sigmaV * dt, 2);
        Q[2][2] = Math.pow(sigmaW * dt, 2);
        Q[3][3] = sigmaBg * sigmaBg * dt;

        double[][] tmp = matMul(matMul(F, s.P), matT(F));
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                s.P[i][j] = tmp[i][j] + Q[i][j];
        symmetrize(s.P);
    }

    static double scalarUpdate(FilterState s, int idx, double z, double R, boolean angleResidual) {
        double innov = z - s.x[idx];
        if (angleResidual) innov = wrapAngle(innov);

        double S = s.P[idx][idx] + R;
        double[] K = new double[4];
        for (int i = 0; i < 4; i++) K[i] = s.P[i][idx] / S;

        for (int i = 0; i < 4; i++) s.x[i] += K[i] * innov;
        s.x[2] = wrapAngle(s.x[2]);

        double[][] Pnew = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Pnew[i][j] = s.P[i][j] - K[i] * s.P[idx][j];
            }
        }
        s.P = Pnew;
        symmetrize(s.P);

        return (innov * innov) / S;
    }

    static boolean isRough(int k) {
        return (300 <= k && k < 520) || (840 <= k && k < 1100) || (1400 <= k && k < 1620);
    }

    static boolean inOutage(int k) {
        boolean o1 = (420 <= k && k < 980);
        boolean o2 = (1250 <= k && k < 1500);
        return o1 || o2;
    }

    static double mean(List<Double> v) {
        if (v.isEmpty()) return Double.NaN;
        double s = 0.0;
        for (double x : v) s += x;
        return s / v.size();
    }

    static double trace4(double[][] A) {
        return A[0][0] + A[1][1] + A[2][2] + A[3][3];
    }

    public static void main(String[] args) {
        final double dt = 0.1;
        final int N = 1800;
        final double bgTrue = 0.015;

        Random rng = new Random(18);

        double[][] xTrue = new double[N][4];
        double[][] xEstHist = new double[N][4];
        double[] pTrace = new double[N];
        List<Double> nisGnss = new ArrayList<>();
        List<Double> nisAnchor = new ArrayList<>();

        FilterState f = new FilterState();
        f.x[0] = 0.0; f.x[1] = 0.0; f.x[2] = 0.05; f.x[3] = 0.0;
        f.P[0][0] = 1.0;
        f.P[1][1] = 1.0;
        f.P[2][2] = Math.pow(5.0 * PI / 180.0, 2);
        f.P[3][3] = Math.pow(0.03, 2);

        for (int k = 1; k < N; k++) {
            double tk = k * dt;
            double vCmd = 1.5 + 0.3 * Math.sin(0.015 * tk);
            double wCmd = 0.08 * Math.sin(0.010 * tk) + 0.04 * Math.sin(0.040 * tk);

            boolean rough = isRough(k);
            double sigmaWTrue = rough ? 0.03 : 0.008;
            double sigmaVTrue = rough ? 0.10 : 0.02;

            double vTrue = vCmd + sigmaVTrue * rng.nextGaussian();
            double wTrue = wCmd + sigmaWTrue * rng.nextGaussian();

            xTrue[k][0] = xTrue[k-1][0] + vTrue * dt * Math.cos(xTrue[k-1][2]);
            xTrue[k][1] = xTrue[k-1][1] + vTrue * dt * Math.sin(xTrue[k-1][2]);
            xTrue[k][2] = wrapAngle(xTrue[k-1][2] + (wTrue - bgTrue) * dt);
            xTrue[k][3] = bgTrue;

            double vm = vTrue + (rough ? 0.12 : 0.03) * rng.nextGaussian();
            double wm = wTrue + (rough ? 0.05 : 0.01) * rng.nextGaussian();

            propagate(f, vm, wm, dt, rough);

            if ((k % 10 == 0) && !inOutage(k)) {
                boolean rtkFixed = (!rough) && (k % 90 != 0);
                double sigmaGps = rtkFixed ? 0.15 : 0.75;

                double zgx = xTrue[k][0] + sigmaGps * rng.nextGaussian();
                double zgy = xTrue[k][1] + sigmaGps * rng.nextGaussian();

                double innovX = zgx - f.x[0];
                double Sx = f.P[0][0] + sigmaGps * sigmaGps;
                double d2x = innovX * innovX / Sx;
                if (d2x < 6.63) {
                    nisGnss.add(scalarUpdate(f, 0, zgx, sigmaGps * sigmaGps, false));
                }

                double innovY = zgy - f.x[1];
                double Sy = f.P[1][1] + sigmaGps * sigmaGps;
                double d2y = innovY * innovY / Sy;
                if (d2y < 6.63) {
                    nisGnss.add(scalarUpdate(f, 1, zgy, sigmaGps * sigmaGps, false));
                }
            }

            if (k == 1000 || k == 1510 || k == 1710) {
                double zax = xTrue[k][0] + 0.10 * rng.nextGaussian();
                double zay = xTrue[k][1] + 0.10 * rng.nextGaussian();
                double zap = wrapAngle(xTrue[k][2] + (2.0 * PI / 180.0) * rng.nextGaussian());

                nisAnchor.add(scalarUpdate(f, 0, zax, 0.10 * 0.10, false));
                nisAnchor.add(scalarUpdate(f, 1, zay, 0.10 * 0.10, false));
                nisAnchor.add(scalarUpdate(f, 2, zap, Math.pow(2.0 * PI / 180.0, 2), true));
            }

            if (nisGnss.size() >= 20) {
                double m = 0.0;
                for (int i = nisGnss.size() - 20; i < nisGnss.size(); i++) m += nisGnss.get(i);
                m /= 20.0;
                if (m > 1.8) {
                    for (int i = 0; i < 4; i++)
                        for (int j = 0; j < 4; j++)
                            f.P[i][j] *= 1.02;
                }
            }

            for (int i = 0; i < 4; i++) xEstHist[k][i] = f.x[i];
            pTrace[k] = trace4(f.P);
        }

        double sumSq = 0.0;
        for (int k = 0; k < N; k++) {
            double ex = xEstHist[k][0] - xTrue[k][0];
            double ey = xEstHist[k][1] - xTrue[k][1];
            sumSq += ex * ex + ey * ey;
        }
        double rmse = Math.sqrt(sumSq / N);

        double exf = xEstHist[N-1][0] - xTrue[N-1][0];
        double eyf = xEstHist[N-1][1] - xTrue[N-1][1];
        double finalErr = Math.sqrt(exf * exf + eyf * eyf);
        double finalHeadingDeg = wrapAngle(xEstHist[N-1][2] - xTrue[N-1][2]) * 180.0 / PI;

        System.out.println("=== Chapter18_Lesson3.java : Long-Range Localization Drift Handling ===");
        System.out.printf("Mission duration: %.1f s%n", N * dt);
        System.out.printf("Position RMSE: %.3f m%n", rmse);
        System.out.printf("Final position error: %.3f m%n", finalErr);
        System.out.printf("Final heading error: %.3f deg%n", finalHeadingDeg);
        System.out.printf("Mean scalar NIS (GNSS accepted): %.3f%n", mean(nisGnss));
        System.out.printf("Mean scalar NIS (Anchor): %.3f%n", mean(nisAnchor));
        System.out.printf("Final covariance trace: %.5f%n", pTrace[N-1]);

        int[][] outages = {{420, 980}, {1250, 1500}};
        String[] names = {"Outage1", "Outage2"};
        for (int i = 0; i < outages.length; i++) {
            int a = outages[i][0];
            int b = outages[i][1];
            double esx = xEstHist[a][0] - xTrue[a][0];
            double esy = xEstHist[a][1] - xTrue[a][1];
            double eex = xEstHist[b-1][0] - xTrue[b-1][0];
            double eey = xEstHist[b-1][1] - xTrue[b-1][1];
            double estart = Math.sqrt(esx * esx + esy * esy);
            double eend = Math.sqrt(eex * eex + eey * eey);
            System.out.printf("%s: error at start=%.3f m, end=%.3f m, growth=%.3f m%n",
                    names[i], estart, eend, eend - estart);
        }
    }
}

// Chapter20_Lesson3.java
// Localization + Mapping Integration (Capstone AMR)
// Pure Java educational implementation of odometry prediction + landmark correction + occupancy log-odds.

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Chapter20_Lesson3 {

    static class Config {
        double dt = 0.1;
        double qv = 0.02;
        double qw = 0.03;
        double rr = 0.15;
        double rb = 0.03;
        double gridRes = 0.20;
        int gridSize = 120;
        double lOcc = 0.85;
        double lFree = -0.40;
        double lMin = -4.0;
        double lMax = 4.0;
    }

    static double wrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static class EKFLocalizer {
        Config cfg;
        double[] x = new double[]{1.0, 1.0, 0.0}; // x, y, theta
        double[][] P = new double[][]{
                {0.05, 0, 0},
                {0, 0.05, 0},
                {0, 0, 0.02}
        };

        EKFLocalizer(Config cfg) {
            this.cfg = cfg;
        }

        void predict(double v, double w) {
            double th = x[2];
            x[0] += v * Math.cos(th) * cfg.dt;
            x[1] += v * Math.sin(th) * cfg.dt;
            x[2] = wrap(x[2] + w * cfg.dt);

            double[][] F = {
                    {1, 0, -v * Math.sin(th) * cfg.dt},
                    {0, 1,  v * Math.cos(th) * cfg.dt},
                    {0, 0, 1}
            };
            double[][] G = {
                    {Math.cos(th) * cfg.dt, 0},
                    {Math.sin(th) * cfg.dt, 0},
                    {0, cfg.dt}
            };
            double[][] Q = {
                    {cfg.qv * cfg.qv, 0},
                    {0, cfg.qw * cfg.qw}
            };
            P = add(mul(F, mul(P, tr(F))), mul(G, mul(Q, tr(G))));
        }

        void correctLandmark(double zRange, double zBearing, double lx, double ly) {
            double dx = lx - x[0];
            double dy = ly - x[1];
            double q = dx * dx + dy * dy;
            if (q < 1e-12) return;

            double zhatRange = Math.sqrt(q);
            double zhatBearing = wrap(Math.atan2(dy, dx) - x[2]);

            double[][] H = {
                    {-dx / Math.sqrt(q), -dy / Math.sqrt(q), 0},
                    { dy / q,            -dx / q,          -1}
            };
            double[][] R = {
                    {cfg.rr * cfg.rr, 0},
                    {0, cfg.rb * cfg.rb}
            };

            double[][] S = add(mul(H, mul(P, tr(H))), R);
            double[] innov = new double[]{zRange - zhatRange, wrap(zBearing - zhatBearing)};
            double[][] SInv = inv2(S);

            double d2 = quadForm(innov, SInv);
            if (d2 > 9.21) return;

            double[][] K = mul(P, mul(tr(H), SInv));
            double[] dxState = mul(K, innov);
            for (int i = 0; i < 3; i++) x[i] += dxState[i];
            x[2] = wrap(x[2]);

            double[][] I = eye3();
            double[][] KH = mul(K, H);
            double[][] IKH = sub(I, KH);
            P = add(mul(IKH, mul(P, tr(IKH))), mul(K, mul(R, tr(K))));
        }
    }

    static class OccupancyGrid {
        Config cfg;
        double[][] L;

        OccupancyGrid(Config cfg) {
            this.cfg = cfg;
            this.L = new double[cfg.gridSize][cfg.gridSize];
        }

        int[] worldToGrid(double x, double y) {
            int gx = (int) (x / cfg.gridRes);
            int gy = (int) (y / cfg.gridRes);
            return new int[]{gx, gy};
        }

        boolean inBounds(int gx, int gy) {
            return gx >= 0 && gy >= 0 && gx < cfg.gridSize && gy < cfg.gridSize;
        }

        List<int[]> bresenham(int x0, int y0, int x1, int y1) {
            List<int[]> pts = new ArrayList<>();
            int dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int err = dx + dy;
            int x = x0, y = y0;
            while (true) {
                pts.add(new int[]{x, y});
                if (x == x1 && y == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x += sx; }
                if (e2 <= dx) { err += dx; y += sy; }
            }
            return pts;
        }

        void updateRay(double rx, double ry, double ex, double ey, boolean hit) {
            int[] g0 = worldToGrid(rx, ry);
            int[] g1 = worldToGrid(ex, ey);
            if (!inBounds(g0[0], g0[1]) || !inBounds(g1[0], g1[1])) return;

            List<int[]> ray = bresenham(g0[0], g0[1], g1[0], g1[1]);
            for (int i = 0; i < ray.size() - 1; i++) {
                int[] p = ray.get(i);
                L[p[1]][p[0]] = clip(L[p[1]][p[0]] + cfg.lFree, cfg.lMin, cfg.lMax);
            }
            if (hit && !ray.isEmpty()) {
                int[] p = ray.get(ray.size() - 1);
                L[p[1]][p[0]] = clip(L[p[1]][p[0]] + cfg.lOcc, cfg.lMin, cfg.lMax);
            }
        }

        void printSummary() {
            double mn = 1.0, mx = 0.0, sum = 0.0;
            int n = cfg.gridSize * cfg.gridSize;
            for (int y = 0; y < cfg.gridSize; y++) {
                for (int x = 0; x < cfg.gridSize; x++) {
                    double p = 1.0 / (1.0 + Math.exp(-L[y][x]));
                    mn = Math.min(mn, p);
                    mx = Math.max(mx, p);
                    sum += p;
                }
            }
            System.out.printf("Map occupancy prob stats: min=%.3f max=%.3f mean=%.3f%n", mn, mx, sum / n);
        }
    }

    // ---------- Linear algebra helpers for small matrices ----------
    static double[][] add(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] + B[i][j];
        return C;
    }
    static double[][] sub(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) for (int j = 0; j < A[0].length; j++) C[i][j] = A[i][j] - B[i][j];
        return C;
    }
    static double[][] tr(double[][] A) {
        double[][] T = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++) for (int j = 0; j < A[0].length; j++) T[j][i] = A[i][j];
        return T;
    }
    static double[][] mul(double[][] A, double[][] B) {
        double[][] C = new double[A.length][B[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int k = 0; k < B.length; k++) {
                for (int j = 0; j < B[0].length; j++) C[i][j] += A[i][k] * B[k][j];
            }
        }
        return C;
    }
    static double[] mul(double[][] A, double[] x) {
        double[] y = new double[A.length];
        for (int i = 0; i < A.length; i++) for (int j = 0; j < x.length; j++) y[i] += A[i][j] * x[j];
        return y;
    }
    static double quadForm(double[] x, double[][] A) {
        double[] t = mul(A, x);
        double s = 0.0;
        for (int i = 0; i < x.length; i++) s += x[i] * t[i];
        return s;
    }
    static double[][] inv2(double[][] A) {
        double a = A[0][0], b = A[0][1], c = A[1][0], d = A[1][1];
        double det = a * d - b * c;
        return new double[][]{{ d / det, -b / det}, {-c / det, a / det}};
    }
    static double[][] eye3() {
        return new double[][]{{1,0,0},{0,1,0},{0,0,1}};
    }
    static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static void main(String[] args) {
        Config cfg = new Config();
        EKFLocalizer ekf = new EKFLocalizer(cfg);
        OccupancyGrid grid = new OccupancyGrid(cfg);

        Random rnd = new Random(5);
        double[] xTrue = new double[]{1.0, 1.0, 0.0};
        double[][] landmarks = {
                {2.0, 10.0}, {10.0, 2.5}, {9.5, 10.5}, {2.0, 5.5}
        };

        double ssePos = 0.0, sseYaw = 0.0;
        int N = 0;

        for (int k = 0; k < 240; k++) {
            double vCmd = 0.35 + 0.05 * Math.sin(0.05 * k);
            double wCmd = 0.25 * Math.sin(0.03 * k);

            xTrue[0] += vCmd * Math.cos(xTrue[2]) * cfg.dt;
            xTrue[1] += vCmd * Math.sin(xTrue[2]) * cfg.dt;
            xTrue[2] = wrap(xTrue[2] + wCmd * cfg.dt + 0.005 * rnd.nextGaussian());

            ekf.predict(vCmd + 0.02 * rnd.nextGaussian(), wCmd + 0.01 * rnd.nextGaussian());

            if (k % 5 == 0) {
                for (double[] lm : landmarks) {
                    double dx = lm[0] - xTrue[0];
                    double dy = lm[1] - xTrue[1];
                    double range = Math.hypot(dx, dy);
                    if (range < 7.5) {
                        double zR = range + 0.08 * rnd.nextGaussian();
                        double zB = wrap(Math.atan2(dy, dx) - xTrue[2] + 0.02 * rnd.nextGaussian());
                        ekf.correctLandmark(zR, zB, lm[0], lm[1]);
                    }
                }
            }

            // Simple mapping update using three synthetic rays
            double[] angs = new double[]{-0.7, 0.0, 0.7};
            for (double a : angs) {
                double rr = 5.0;
                double ex = ekf.x[0] + rr * Math.cos(ekf.x[2] + a);
                double ey = ekf.x[1] + rr * Math.sin(ekf.x[2] + a);
                grid.updateRay(ekf.x[0], ekf.x[1], ex, ey, true);
            }

            double ex = ekf.x[0] - xTrue[0];
            double ey = ekf.x[1] - xTrue[1];
            ssePos += ex * ex + ey * ey;
            sseYaw += Math.pow(wrap(ekf.x[2] - xTrue[2]), 2);
            N++;
        }

        System.out.printf("Final estimated pose: [%.4f, %.4f, %.4f]%n", ekf.x[0], ekf.x[1], ekf.x[2]);
        System.out.printf("Final covariance diag: [%.5f, %.5f, %.5f]%n", ekf.P[0][0], ekf.P[1][1], ekf.P[2][2]);
        System.out.printf("Position RMSE [m]: %.4f%n", Math.sqrt(ssePos / N));
        System.out.printf("Yaw RMSE [rad]: %.4f%n", Math.sqrt(sseYaw / N));
        grid.printSummary();
    }
}

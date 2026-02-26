// Chapter19_Lesson1.java
// Metrics for Localization and SLAM (Autonomous Mobile Robots)
// Java reference implementation: SE(2) alignment, ATE, RPE (2D)

import java.util.Random;

public class Chapter19_Lesson1 {

    static class Pose2 {
        double x, y, yaw;
        Pose2(double x, double y, double yaw) {
            this.x = x; this.y = y; this.yaw = yaw;
        }
    }

    static class AlignResult {
        double[][] R;   // 2x2
        double[] t;     // 2x1
        double theta;
        AlignResult(double[][] R, double[] t, double theta) {
            this.R = R; this.t = t; this.theta = theta;
        }
    }

    static double wrapToPi(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    static AlignResult alignSE2(Pose2[] gt, Pose2[] est) {
        int N = gt.length;
        double cgx = 0, cgy = 0, cex = 0, cey = 0;
        for (int i = 0; i < N; i++) {
            cgx += gt[i].x; cgy += gt[i].y;
            cex += est[i].x; cey += est[i].y;
        }
        cgx /= N; cgy /= N; cex /= N; cey /= N;

        double s = 0.0, c = 0.0;
        for (int i = 0; i < N; i++) {
            double Xx = est[i].x - cex, Xy = est[i].y - cey;
            double Yx = gt[i].x - cgx,  Yy = gt[i].y - cgy;
            s += Xx * Yy - Xy * Yx;
            c += Xx * Yx + Xy * Yy;
        }
        double theta = Math.atan2(s, c);
        double ct = Math.cos(theta), st = Math.sin(theta);

        double[][] R = new double[][] {
            { ct, -st },
            { st,  ct }
        };
        double[] t = new double[] {
            cgx - (R[0][0] * cex + R[0][1] * cey),
            cgy - (R[1][0] * cex + R[1][1] * cey)
        };
        return new AlignResult(R, t, theta);
    }

    static Pose2[] applySE2(Pose2[] est, AlignResult A) {
        Pose2[] out = new Pose2[est.length];
        for (int i = 0; i < est.length; i++) {
            double x = est[i].x, y = est[i].y;
            double xa = A.R[0][0] * x + A.R[0][1] * y + A.t[0];
            double ya = A.R[1][0] * x + A.R[1][1] * y + A.t[1];
            out[i] = new Pose2(xa, ya, wrapToPi(est[i].yaw + A.theta));
        }
        return out;
    }

    static double ateRMSE(Pose2[] gt, Pose2[] est) {
        double s = 0.0;
        for (int i = 0; i < gt.length; i++) {
            double dx = gt[i].x - est[i].x;
            double dy = gt[i].y - est[i].y;
            s += dx * dx + dy * dy;
        }
        return Math.sqrt(s / gt.length);
    }

    static double[] rpeRMSE(Pose2[] gt, Pose2[] est, int delta) {
        int M = gt.length - delta;
        double sT = 0.0, sR = 0.0;
        for (int k = 0; k < M; k++) {
            double dgx = gt[k + delta].x - gt[k].x;
            double dgy = gt[k + delta].y - gt[k].y;
            double dex = est[k + delta].x - est[k].x;
            double dey = est[k + delta].y - est[k].y;
            double dt = Math.hypot(dgx - dex, dgy - dey);
            sT += dt * dt;

            double dyg = wrapToPi(gt[k + delta].yaw - gt[k].yaw);
            double dye = wrapToPi(est[k + delta].yaw - est[k].yaw);
            double dr = Math.abs(wrapToPi(dyg - dye));
            sR += dr * dr;
        }
        return new double[] { Math.sqrt(sT / M), Math.sqrt(sR / M) };
    }

    public static void main(String[] args) {
        Random rng = new Random(19);
        int N = 200;
        Pose2[] gt = new Pose2[N];
        Pose2[] est = new Pose2[N];

        double theta0 = Math.toRadians(7.0);
        double c0 = Math.cos(theta0), s0 = Math.sin(theta0);

        double[] tx = new double[N];
        double[] ty = new double[N];

        for (int i = 0; i < N; i++) {
            double tt = 20.0 * i / (N - 1.0);
            tx[i] = 0.5 * tt;
            ty[i] = 2.0 * Math.sin(0.4 * tt);
        }

        for (int i = 0; i < N; i++) {
            double dx = (i == 0) ? tx[1] - tx[0] : tx[i] - tx[i - 1];
            double dy = (i == 0) ? ty[1] - ty[0] : ty[i] - ty[i - 1];
            double yaw = Math.atan2(dy, dx);
            gt[i] = new Pose2(tx[i], ty[i], yaw);

            // inverse global transform + drift + noise
            double gx = gt[i].x - 1.5;
            double gy = gt[i].y + 0.8;
            double ex0 =  c0 * gx + s0 * gy;
            double ey0 = -s0 * gx + c0 * gy;

            double tt = 20.0 * i / (N - 1.0);
            double ex = ex0 + 0.01 * tt + 0.03 * rng.nextGaussian();
            double ey = ey0 - 0.004 * tt + 0.03 * rng.nextGaussian();
            double eyaw = wrapToPi(yaw - theta0 + 0.01 * Math.sin(0.2 * tt) + 0.01 * rng.nextGaussian());
            est[i] = new Pose2(ex, ey, eyaw);
        }

        AlignResult A = alignSE2(gt, est);
        Pose2[] estA = applySE2(est, A);
        double ate = ateRMSE(gt, estA);
        double[] rpe = rpeRMSE(gt, estA, 5);

        System.out.println("=== Chapter19 Lesson1 Metrics Demo (Java) ===");
        System.out.printf("ATE RMSE [m]: %.6f%n", ate);
        System.out.printf("RPE translational RMSE [m] (delta=5): %.6f%n", rpe[0]);
        System.out.printf("RPE rotational RMSE [rad] (delta=5): %.6f%n", rpe[1]);
        System.out.printf("Estimated global alignment theta [deg]: %.4f%n", Math.toDegrees(A.theta));
    }
}

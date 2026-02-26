// Chapter8_Lesson4.java
// Particle-Filter Localization — Degeneracy + Kidnapped Robot Recovery (2D)
//
// Compile:
//   javac Chapter8_Lesson4.java
// Run:
//   java Chapter8_Lesson4
//
// Output:
//   Chapter8_Lesson4_results.csv

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter8_Lesson4 {

    static class Pose {
        double x, y, th;
        Pose(double x, double y, double th) { this.x=x; this.y=y; this.th=th; }
    }

    static class Particle {
        double x, y, th, w;
        Particle(double x, double y, double th, double w) { this.x=x; this.y=y; this.th=th; this.w=w; }
        Particle copy() { return new Particle(x,y,th,w); }
    }

    static final double PI = Math.PI;

    static double wrapAngle(double a) {
        a = (a + PI) % (2.0 * PI);
        if (a < 0) a += 2.0 * PI;
        return a - PI;
    }

    static double logGauss0(double e, double sigma) {
        return -0.5 * (e/sigma)*(e/sigma) - Math.log(Math.sqrt(2.0 * PI) * sigma);
    }

    static Particle motionStep(Particle p, double v, double w, double dt,
                               double sigmaV, double sigmaW, Random rng) {
        double vN = v + sigmaV * rng.nextGaussian();
        double wN = w + sigmaW * rng.nextGaussian();
        double x2, y2, th2;

        if (Math.abs(wN) < 1e-9) {
            x2 = p.x + vN * dt * Math.cos(p.th);
            y2 = p.y + vN * dt * Math.sin(p.th);
            th2 = p.th;
        } else {
            x2 = p.x + (vN / wN) * (Math.sin(p.th + wN * dt) - Math.sin(p.th));
            y2 = p.y - (vN / wN) * (Math.cos(p.th + wN * dt) - Math.cos(p.th));
            th2 = p.th + wN * dt;
        }
        th2 = wrapAngle(th2);
        return new Particle(x2, y2, th2, p.w);
    }

    static void measurementModel(Pose x, double[][] landmarks, double[][] outRB) {
        for (int i = 0; i < landmarks.length; i++) {
            double dx = landmarks[i][0] - x.x;
            double dy = landmarks[i][1] - x.y;
            double r = Math.sqrt(dx*dx + dy*dy);
            double b = Math.atan2(dy, dx) - x.th;
            b = wrapAngle(b);
            outRB[i][0] = r;
            outRB[i][1] = b;
        }
    }

    static double logSensorLikelihood(Particle p, double[][] z, double[][] landmarks,
                                      double sigmaR, double sigmaB) {
        double ll = 0.0;
        for (int i = 0; i < landmarks.length; i++) {
            double dx = landmarks[i][0] - p.x;
            double dy = landmarks[i][1] - p.y;
            double rHat = Math.sqrt(dx*dx + dy*dy);
            double bHat = wrapAngle(Math.atan2(dy, dx) - p.th);

            double dr = z[i][0] - rHat;
            double db = wrapAngle(z[i][1] - bHat);

            ll += logGauss0(dr, sigmaR);
            ll += logGauss0(db, sigmaB);
        }
        return ll;
    }

    static void normalize(Particle[] P) {
        double s = 0.0;
        for (Particle p : P) s += p.w;
        if (!(s > 0.0) || !Double.isFinite(s)) {
            double w = 1.0 / P.length;
            for (Particle p : P) p.w = w;
            return;
        }
        for (Particle p : P) p.w /= s;
    }

    static double Neff(Particle[] P) {
        double s2 = 0.0;
        for (Particle p : P) s2 += p.w * p.w;
        if (s2 <= 0.0) return 0.0;
        return 1.0 / s2;
    }

    static Particle[] systematicResample(Particle[] P, Random rng) {
        int N = P.length;
        double[] cdf = new double[N];
        double acc = 0.0;
        for (int i = 0; i < N; i++) {
            acc += P[i].w;
            cdf[i] = acc;
        }
        cdf[N-1] = 1.0;

        double u0 = rng.nextDouble() / N;

        Particle[] out = new Particle[N];
        int i = 0;
        for (int m = 0; m < N; m++) {
            double u = u0 + (double)m / N;
            while (u > cdf[i]) i++;
            Particle q = P[i].copy();
            q.w = 1.0 / N;
            out[m] = q;
        }
        return out;
    }

    static void roughen(Particle[] P, double k,
                        double xmin, double xmax, double ymin, double ymax,
                        Random rng) {
        int N = P.length;
        double d = 3.0;
        double sx = k * (xmax - xmin) * Math.pow((double)N, -1.0/d);
        double sy = k * (ymax - ymin) * Math.pow((double)N, -1.0/d);
        double sth = k * (2.0 * PI) * Math.pow((double)N, -1.0/d);

        for (Particle p : P) {
            p.x += sx * rng.nextGaussian();
            p.y += sy * rng.nextGaussian();
            p.th = wrapAngle(p.th + sth * rng.nextGaussian());
        }
    }

    static void injectRandom(Particle[] P, double frac,
                             double xmin, double xmax, double ymin, double ymax,
                             Random rng) {
        int N = P.length;
        int m = (int)Math.round(Math.max(0.0, Math.min(1.0, frac)) * N);
        if (m <= 0) return;
        for (int i = 0; i < m; i++) {
            P[i].x = xmin + (xmax - xmin) * rng.nextDouble();
            P[i].y = ymin + (ymax - ymin) * rng.nextDouble();
            P[i].th = -PI + (2.0 * PI) * rng.nextDouble();
        }
    }

    static Pose estimate(Particle[] P) {
        double x=0.0, y=0.0;
        double c=0.0, s=0.0;
        for (Particle p : P) {
            x += p.w * p.x;
            y += p.w * p.y;
            c += p.w * Math.cos(p.th);
            s += p.w * Math.sin(p.th);
        }
        return new Pose(x, y, Math.atan2(s, c));
    }

    static double[] control(int t) {
        if (t < 70)  return new double[]{0.7, 0.0};
        if (t < 90)  return new double[]{0.7, 0.9};
        if (t < 160) return new double[]{0.7, 0.0};
        if (t < 180) return new double[]{0.7, 0.9};
        if (t < 250) return new double[]{0.7, 0.0};
        return new double[]{0.7, 0.9};
    }

    public static void main(String[] args) throws IOException {
        Random rng = new Random(4);

        double xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0;

        double[][] landmarks = new double[][]{
                {2.0,2.0},{8.0,2.0},{8.0,8.0},{2.0,8.0}
        };

        double dt = 0.1;
        int T = 300;
        int kidnappedT = 170;

        double sigmaV = 0.05, sigmaW = 0.03;
        double sigmaR = 0.15, sigmaB = 0.07;

        int N = 800;
        double NeffRatio = 0.5;
        double roughK = 0.15;

        double epsMin = 0.01, epsMax = 0.30;
        double llThresh = -12.0;

        Pose xTrue = new Pose(1.0, 1.0, 0.0);

        Particle[] P = new Particle[N];
        for (int i = 0; i < N; i++) {
            P[i] = new Particle(
                    xmin + (xmax - xmin) * rng.nextDouble(),
                    ymin + (ymax - ymin) * rng.nextDouble(),
                    -PI + (2.0 * PI) * rng.nextDouble(),
                    1.0 / N
            );
        }

        double[][] z = new double[landmarks.length][2];
        double[][] zHat = new double[landmarks.length][2];

        try (FileWriter fw = new FileWriter("Chapter8_Lesson4_results.csv")) {
            fw.write("t,true_x,true_y,true_th,est_x,est_y,est_th,Neff,eps\n");

            for (int t = 0; t < T; t++) {
                double[] uw = control(t);
                double vCmd = uw[0], wCmd = uw[1];

                Particle tmp = new Particle(xTrue.x, xTrue.y, xTrue.th, 1.0);
                tmp = motionStep(tmp, vCmd, wCmd, dt, sigmaV, sigmaW, rng);
                xTrue = new Pose(tmp.x, tmp.y, tmp.th);

                if (t == kidnappedT) {
                    xTrue = new Pose(
                            xmin + (xmax - xmin) * rng.nextDouble(),
                            ymin + (ymax - ymin) * rng.nextDouble(),
                            -PI + (2.0 * PI) * rng.nextDouble()
                    );
                }

                measurementModel(xTrue, landmarks, zHat);
                for (int i = 0; i < landmarks.length; i++) {
                    z[i][0] = zHat[i][0] + sigmaR * rng.nextGaussian();
                    z[i][1] = wrapAngle(zHat[i][1] + sigmaB * rng.nextGaussian());
                }

                for (int i = 0; i < N; i++) {
                    Particle q = motionStep(P[i], vCmd, wCmd, dt, sigmaV, sigmaW, rng);
                    P[i].x = q.x; P[i].y = q.y; P[i].th = q.th;
                }

                double[] lls = new double[N];
                double llMax = -1e300;
                for (int i = 0; i < N; i++) {
                    lls[i] = logSensorLikelihood(P[i], z, landmarks, sigmaR, sigmaB);
                    if (lls[i] > llMax) llMax = lls[i];
                }
                for (int i = 0; i < N; i++) {
                    P[i].w = Math.exp(lls[i] - llMax);
                }
                normalize(P);

                double lbar = 0.0;
                for (int i = 0; i < N; i++) {
                    lbar += P[i].w * logSensorLikelihood(P[i], z, landmarks, sigmaR, sigmaB);
                }
                double eps = (lbar < llThresh) ? epsMax : epsMin;

                double neff = Neff(P);
                if (neff < NeffRatio * N) {
                    P = systematicResample(P, rng);
                    roughen(P, roughK, xmin, xmax, ymin, ymax, rng);
                    injectRandom(P, eps, xmin, xmax, ymin, ymax, rng);

                    llMax = -1e300;
                    for (int i = 0; i < N; i++) {
                        lls[i] = logSensorLikelihood(P[i], z, landmarks, sigmaR, sigmaB);
                        if (lls[i] > llMax) llMax = lls[i];
                    }
                    for (int i = 0; i < N; i++) {
                        P[i].w = Math.exp(lls[i] - llMax);
                    }
                    normalize(P);
                }

                Pose xEst = estimate(P);
                fw.write(String.format("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                        t, xTrue.x, xTrue.y, xTrue.th, xEst.x, xEst.y, xEst.th, neff, eps));
            }
        }

        System.out.println("Saved: Chapter8_Lesson4_results.csv");
    }
}

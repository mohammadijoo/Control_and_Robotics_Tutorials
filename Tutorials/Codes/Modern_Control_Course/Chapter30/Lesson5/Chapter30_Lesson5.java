// Chapter30_Lesson5.java
//
// Bridge lab for Modern Control, Chapter 30 Lesson 5.
// Implements an unconstrained finite-horizon MPC/LQR recursion from scratch
// for a sampled double integrator.
//
// Compile:
//   javac Chapter30_Lesson5.java
//
// Run:
//   java Chapter30_Lesson5

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Chapter30_Lesson5 {
    static class Vec2 {
        double x1, x2;
        Vec2(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static class Mat2 {
        double a11, a12, a21, a22;
        Mat2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class GainResult {
        Vec2 K0;
        List<Vec2> gains;
        GainResult(Vec2 K0, List<Vec2> gains) {
            this.K0 = K0;
            this.gains = gains;
        }
    }

    static Vec2 add(Vec2 a, Vec2 b) {
        return new Vec2(a.x1 + b.x1, a.x2 + b.x2);
    }

    static Vec2 scale(Vec2 a, double s) {
        return new Vec2(s * a.x1, s * a.x2);
    }

    static Vec2 matVec(Mat2 A, Vec2 x) {
        return new Vec2(
            A.a11 * x.x1 + A.a12 * x.x2,
            A.a21 * x.x1 + A.a22 * x.x2
        );
    }

    static Mat2 add(Mat2 A, Mat2 B) {
        return new Mat2(
            A.a11 + B.a11, A.a12 + B.a12,
            A.a21 + B.a21, A.a22 + B.a22
        );
    }

    static Mat2 sub(Mat2 A, Mat2 B) {
        return new Mat2(
            A.a11 - B.a11, A.a12 - B.a12,
            A.a21 - B.a21, A.a22 - B.a22
        );
    }

    static Mat2 mul(Mat2 A, Mat2 B) {
        return new Mat2(
            A.a11 * B.a11 + A.a12 * B.a21,
            A.a11 * B.a12 + A.a12 * B.a22,
            A.a21 * B.a11 + A.a22 * B.a21,
            A.a21 * B.a12 + A.a22 * B.a22
        );
    }

    static Mat2 transpose(Mat2 A) {
        return new Mat2(A.a11, A.a21, A.a12, A.a22);
    }

    static double dot(Vec2 a, Vec2 b) {
        return a.x1 * b.x1 + a.x2 * b.x2;
    }

    static Mat2 outer(Vec2 a, Vec2 b) {
        return new Mat2(
            a.x1 * b.x1, a.x1 * b.x2,
            a.x2 * b.x1, a.x2 * b.x2
        );
    }

    static GainResult finiteHorizonLqr(Mat2 Ad, Vec2 Bd, Mat2 Q, double R,
                                       Mat2 Qf, int horizon) {
        Mat2 P = Qf;
        List<Vec2> backward = new ArrayList<>();

        for (int k = horizon - 1; k >= 0; --k) {
            Vec2 PB = matVec(P, Bd);
            double S = R + dot(Bd, PB);

            Vec2 PAcol1 = matVec(P, new Vec2(Ad.a11, Ad.a21));
            Vec2 PAcol2 = matVec(P, new Vec2(Ad.a12, Ad.a22));

            Vec2 K = new Vec2(
                (Bd.x1 * PAcol1.x1 + Bd.x2 * PAcol1.x2) / S,
                (Bd.x1 * PAcol2.x1 + Bd.x2 * PAcol2.x2) / S
            );
            backward.add(K);

            Mat2 BK = outer(Bd, K);
            Mat2 AdMinusBK = sub(Ad, BK);
            P = add(Q, mul(transpose(Ad), mul(P, AdMinusBK)));
        }

        Collections.reverse(backward);
        return new GainResult(backward.get(0), backward);
    }

    public static void main(String[] args) {
        double Ts = 0.05;

        Mat2 Ad = new Mat2(1.0, Ts, 0.0, 1.0);
        Vec2 Bd = new Vec2(0.5 * Ts * Ts, Ts);

        Mat2 Q = new Mat2(10.0, 0.0, 0.0, 1.0);
        double R = 0.25;
        Mat2 Qf = new Mat2(10.0, 0.0, 0.0, 1.0);

        GainResult result = finiteHorizonLqr(Ad, Bd, Q, R, Qf, 25);
        Vec2 K = result.K0;

        System.out.printf("Finite-horizon MPC first gain K0 = [%.6f, %.6f]%n",
                          K.x1, K.x2);

        Vec2 x = new Vec2(1.0, 0.0);
        double cost = 0.0;

        for (int k = 0; k < 80; ++k) {
            double u = -(K.x1 * x.x1 + K.x2 * x.x2);
            double stage = 10.0 * x.x1 * x.x1 + x.x2 * x.x2 + R * u * u;
            cost += stage;

            if (k % 10 == 0) {
                System.out.printf("k=%2d  x=[%10.6f, %10.6f]  u=%10.6f%n",
                                  k, x.x1, x.x2, u);
            }

            x = add(matVec(Ad, x), scale(Bd, u));
        }

        System.out.printf("Approximate accumulated cost = %.6f%n", cost);
        System.out.printf("Final state = [%.6f, %.6f]%n", x.x1, x.x2);
    }
}

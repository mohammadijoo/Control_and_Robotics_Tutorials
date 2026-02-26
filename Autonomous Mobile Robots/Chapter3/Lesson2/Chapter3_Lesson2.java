/*
Chapter 3 - Lesson 2: Curvature and Turning Radius Limits (Java)

This program:
  1) Computes signed curvature at interior points of a polyline using the 3-point formula.
  2) Checks steering curvature limit kappa_max = tan(delta_max)/L.

No external libraries are required.
*/

import java.util.ArrayList;
import java.util.List;

public class Chapter3_Lesson2 {

    static class Vec2 {
        final double x;
        final double y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 minus(Vec2 o) { return new Vec2(this.x - o.x, this.y - o.y); }
        double norm() { return Math.sqrt(x*x + y*y); }
    }

    static double signedCurvatureThreePoints(Vec2 p0, Vec2 p1, Vec2 p2) {
        Vec2 v01 = p1.minus(p0);
        Vec2 v02 = p2.minus(p0);
        Vec2 v12 = p2.minus(p1);

        double a = v12.norm();
        double b = v02.norm();
        double c = v01.norm();

        // Signed double area: cross(v01, v02)
        double area2 = v01.x * v02.y - v01.y * v02.x;

        double denom = a * b * c;
        if (denom < 1e-12) return 0.0;
        return 2.0 * area2 / denom;
    }

    static double[] curvaturePolyline(List<Vec2> P) {
        int N = P.size();
        double[] kappa = new double[N];
        for (int i = 1; i < N - 1; i++) {
            kappa[i] = signedCurvatureThreePoints(P.get(i-1), P.get(i), P.get(i+1));
        }
        // endpoints remain 0
        return kappa;
    }

    static double kappaMaxSteer(double L, double deltaMaxRad) {
        return Math.tan(deltaMaxRad) / L;
    }

    public static void main(String[] args) {
        int N = 401;
        List<Vec2> P = new ArrayList<>(N);
        for (int i = 0; i < N; i++) {
            double s = 12.0 * ((double)i) / ((double)(N - 1));
            double x = s;
            double y = 1.2 * Math.sin(0.7 * s) + 0.2 * Math.sin(2.2 * s);
            P.add(new Vec2(x, y));
        }

        double[] kappa = curvaturePolyline(P);

        double L = 0.35;
        double deltaMax = Math.toRadians(28.0);
        double kappaMax = kappaMaxSteer(L, deltaMax);

        boolean ok = true;
        double kappaPeak = 0.0;
        for (double ki : kappa) {
            kappaPeak = Math.max(kappaPeak, Math.abs(ki));
            if (Math.abs(ki) > kappaMax + 1e-12) ok = false;
        }

        System.out.println("kappa_max_steer = " + kappaMax + " 1/m");
        System.out.println("peak |kappa|    = " + kappaPeak + " 1/m");
        System.out.println("steering-feasible? " + (ok ? "YES" : "NO"));
    }
}

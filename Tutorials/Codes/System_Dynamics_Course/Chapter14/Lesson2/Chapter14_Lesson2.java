/*
Chapter14_Lesson2.java
Equilibria, local classification, and RK4 simulation for a planar autonomous system.

Compile:
  javac Chapter14_Lesson2.java
Run:
  java Chapter14_Lesson2
Outputs:
  trajectories_java.csv
*/

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chapter14_Lesson2 {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
    }

    static class LinClass {
        String kind;
        double tr, det, disc;
        LinClass(String kind, double tr, double det, double disc) {
            this.kind = kind; this.tr = tr; this.det = det; this.disc = disc;
        }
    }

    static Vec2 F(double t, Vec2 z, double a) {
        // Example system:
        // x' = x - x^3 - y
        // y' = x + a y
        double dx = z.x - z.x*z.x*z.x - z.y;
        double dy = z.x + a*z.y;
        return new Vec2(dx, dy);
    }

    static double[][] jacobian(Vec2 z, double a) {
        // Analytical Jacobian
        double[][] J = new double[2][2];
        J[0][0] = 1.0 - 3.0*z.x*z.x;
        J[0][1] = -1.0;
        J[1][0] = 1.0;
        J[1][1] = a;
        return J;
    }

    static LinClass classify2x2(double[][] J, double eps) {
        double tr  = J[0][0] + J[1][1];
        double det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
        double disc = tr*tr - 4.0*det;

        String kind;
        if (det < -eps) {
            kind = "saddle (hyperbolic)";
        } else if (Math.abs(det) <= eps) {
            kind = "degenerate (det ~ 0)";
        } else {
            if (disc > eps) {
                if (tr < -eps) kind = "stable node";
                else if (tr > eps) kind = "unstable node";
                else kind = "improper/star node";
            } else if (disc < -eps) {
                if (tr < -eps) kind = "stable spiral (focus)";
                else if (tr > eps) kind = "unstable spiral (focus)";
                else kind = "center (linear); nonlinear decides";
            } else {
                if (tr < -eps) kind = "stable degenerate node";
                else if (tr > eps) kind = "unstable degenerate node";
                else kind = "degenerate/center";
            }
        }
        return new LinClass(kind, tr, det, disc);
    }

    static Vec2 rk4Step(double t, Vec2 z, double h, double a) {
        Vec2 k1 = F(t, z, a);
        Vec2 k2 = F(t + 0.5*h, new Vec2(z.x + 0.5*h*k1.x, z.y + 0.5*h*k1.y), a);
        Vec2 k3 = F(t + 0.5*h, new Vec2(z.x + 0.5*h*k2.x, z.y + 0.5*h*k2.y), a);
        Vec2 k4 = F(t + h, new Vec2(z.x + h*k3.x, z.y + h*k3.y), a);

        double dx = (k1.x + 2.0*k2.x + 2.0*k3.x + k4.x) * (h/6.0);
        double dy = (k1.y + 2.0*k2.y + 2.0*k3.y + k4.y) * (h/6.0);
        return new Vec2(z.x + dx, z.y + dy);
    }

    public static void main(String[] args) throws IOException {
        final double a = 1.0;

        // Analytical equilibria for a>0
        double xp = Math.sqrt((a+1.0)/a);

        List<Vec2> eqs = new ArrayList<>();
        eqs.add(new Vec2(0.0, 0.0));
        eqs.add(new Vec2(+xp, -xp));
        eqs.add(new Vec2(-xp, +xp));

        System.out.println("Equilibria and local classification (linearization):");
        for (Vec2 z : eqs) {
            double[][] J = jacobian(z, a);
            LinClass cls = classify2x2(J, 1e-12);
            System.out.printf("  z*=(%.6f, %.6f)  trace=%.6f det=%.6f -> %s%n",
                    z.x, z.y, cls.tr, cls.det, cls.kind);
        }

        // Simulate trajectories and write CSV
        List<Vec2> initials = List.of(
                new Vec2(-2.0, -2.0), new Vec2(-2.0, 0.0), new Vec2(-2.0, 2.0),
                new Vec2( 0.5, -2.0), new Vec2( 0.5, 2.0),
                new Vec2( 2.0, -2.0), new Vec2( 2.0, 0.0), new Vec2( 2.0, 2.0)
        );

        double h = 0.01;
        double tmax = 12.0;
        int steps = (int)Math.round(tmax / h);

        try (FileWriter fw = new FileWriter("trajectories_java.csv")) {
            fw.write("traj_id,t,x,y\n");
            for (int k = 0; k < initials.size(); k++) {
                Vec2 z = new Vec2(initials.get(k).x, initials.get(k).y);
                double t = 0.0;
                for (int i = 0; i <= steps; i++) {
                    fw.write(k + "," + t + "," + z.x + "," + z.y + "\n");
                    z = rk4Step(t, z, h, a);
                    t += h;
                }
            }
        }

        System.out.println("\nWrote trajectories_java.csv");
    }
}

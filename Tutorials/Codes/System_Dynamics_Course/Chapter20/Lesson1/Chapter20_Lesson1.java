// Chapter20_Lesson1.java
/*
System Dynamics — Chapter 20, Lesson 1
Nonlinear Maps and Continuous-Time Chaotic Systems (Logistic Map, Lorenz System)

This program:
1) Simulates logistic map and writes logistic.csv.
2) Integrates Lorenz system via RK4 and writes lorenz.csv.

Compile & run:
  javac Chapter20_Lesson1.java
  java Chapter20_Lesson1

Outputs:
  logistic.csv  (n, x_n)
  lorenz.csv    (t, x, y, z)
*/

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class Chapter20_Lesson1 {

    // Logistic map x_{n+1} = r x_n (1 - x_n)
    static double[] logisticMap(double r, double x0, int n) {
        double[] x = new double[n + 1];
        x[0] = x0;
        for (int k = 0; k < n; k++) {
            x[k + 1] = r * x[k] * (1.0 - x[k]);
        }
        return x;
    }

    static class Vec3 {
        double x, y, z;
        Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        Vec3 add(Vec3 b) { return new Vec3(this.x + b.x, this.y + b.y, this.z + b.z); }
        Vec3 mul(double s) { return new Vec3(s * this.x, s * this.y, s * this.z); }
    }

    static Vec3 lorenzRHS(double t, Vec3 s, double sigma, double rho, double beta) {
        double dx = sigma * (s.y - s.x);
        double dy = s.x * (rho - s.z) - s.y;
        double dz = s.x * s.y - beta * s.z;
        return new Vec3(dx, dy, dz);
    }

    static Vec3 rk4Step(double t, Vec3 x, double h, double sigma, double rho, double beta) {
        Vec3 k1 = lorenzRHS(t, x, sigma, rho, beta);
        Vec3 k2 = lorenzRHS(t + 0.5 * h, x.add(k1.mul(0.5 * h)), sigma, rho, beta);
        Vec3 k3 = lorenzRHS(t + 0.5 * h, x.add(k2.mul(0.5 * h)), sigma, rho, beta);
        Vec3 k4 = lorenzRHS(t + h, x.add(k3.mul(h)), sigma, rho, beta);
        return x.add(k1.mul(h / 6.0))
                .add(k2.mul(h / 3.0))
                .add(k3.mul(h / 3.0))
                .add(k4.mul(h / 6.0));
    }

    public static void main(String[] args) throws IOException {
        // Logistic map
        double r = 3.8;
        double x0 = 0.2;
        int n = 2000;
        double[] x = logisticMap(r, x0, n);

        try (BufferedWriter w = new BufferedWriter(new FileWriter("logistic.csv"))) {
            w.write("n,x\n");
            for (int k = 0; k <= n; k++) {
                w.write(k + "," + x[k] + "\n");
            }
        }

        // Lorenz RK4
        double sigma = 10.0;
        double rho = 28.0;
        double beta = 8.0 / 3.0;

        double t0 = 0.0;
        double tf = 40.0;
        double h = 0.005;
        int steps = (int) Math.ceil((tf - t0) / h);

        Vec3 s = new Vec3(1.0, 1.0, 1.0);
        double t = t0;

        try (BufferedWriter w = new BufferedWriter(new FileWriter("lorenz.csv"))) {
            w.write("t,x,y,z\n");
            w.write(t + "," + s.x + "," + s.y + "," + s.z + "\n");
            for (int k = 0; k < steps; k++) {
                s = rk4Step(t, s, h, sigma, rho, beta);
                t += h;
                w.write(t + "," + s.x + "," + s.y + "," + s.z + "\n");
            }
        }

        System.out.println("Wrote logistic.csv and lorenz.csv");
    }
}

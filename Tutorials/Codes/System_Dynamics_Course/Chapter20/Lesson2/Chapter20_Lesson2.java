// Chapter20_Lesson2.java
// System Dynamics — Chapter 20 (Chaos, Complex Dynamics, and Computational Tools)
// Lesson 2: Bifurcation Diagrams, Period Doubling, and Strange Attractors
//
// Compile:
//   javac Chapter20_Lesson2.java
// Run:
//   java Chapter20_Lesson2
//
// Outputs:
//   logistic_bifurcation_java.csv
//   lorenz_traj_java.csv

import java.io.FileWriter;
import java.io.IOException;

public class Chapter20_Lesson2 {

    static double logistic(double x, double r) {
        return r * x * (1.0 - x);
    }

    static void bifurcationLogistic(double rmin, double rmax, int Nr,
                                   int nTransient, int nKeep, double x0) throws IOException {
        double[] rs = new double[Nr];
        double[] x = new double[Nr];
        for (int i = 0; i < Nr; i++) {
            rs[i] = rmin + (rmax - rmin) * (double)i / (double)(Nr - 1);
            x[i] = x0;
        }

        for (int k = 0; k < nTransient; k++) {
            for (int i = 0; i < Nr; i++) x[i] = logistic(x[i], rs[i]);
        }

        FileWriter fw = new FileWriter("logistic_bifurcation_java.csv");
        fw.write("r,x\n");
        for (int k = 0; k < nKeep; k++) {
            for (int i = 0; i < Nr; i++) x[i] = logistic(x[i], rs[i]);
            for (int i = 0; i < Nr; i++) fw.write(rs[i] + "," + x[i] + "\n");
        }
        fw.close();
    }

    static class Vec3 {
        double x, y, z;
        Vec3(double x, double y, double z){ this.x=x; this.y=y; this.z=z; }
        Vec3 add(Vec3 o){ return new Vec3(x+o.x, y+o.y, z+o.z); }
        Vec3 mul(double a){ return new Vec3(a*x, a*y, a*z); }
    }

    static Vec3 lorenzRhs(Vec3 s, double sigma, double rho, double beta) {
        return new Vec3(
            sigma*(s.y - s.x),
            s.x*(rho - s.z) - s.y,
            s.x*s.y - beta*s.z
        );
    }

    static Vec3 rk4Step(Vec3 y, double h, double sigma, double rho, double beta) {
        Vec3 k1 = lorenzRhs(y, sigma, rho, beta);
        Vec3 k2 = lorenzRhs(y.add(k1.mul(0.5*h)), sigma, rho, beta);
        Vec3 k3 = lorenzRhs(y.add(k2.mul(0.5*h)), sigma, rho, beta);
        Vec3 k4 = lorenzRhs(y.add(k3.mul(h)), sigma, rho, beta);
        return y.add(k1.add(k2.mul(2.0)).add(k3.mul(2.0)).add(k4).mul(h/6.0));
    }

    static void simulateLorenz(double T, double h, Vec3 y0,
                              double sigma, double rho, double beta,
                              double transient, int stride) throws IOException {
        int N = (int)(T / h);
        double t = 0.0;
        Vec3 y = y0;

        FileWriter fw = new FileWriter("lorenz_traj_java.csv");
        fw.write("t,x,y,z\n");
        for (int i = 0; i < N; i++) {
            y = rk4Step(y, h, sigma, rho, beta);
            t += h;
            if (t >= transient && (i % stride == 0)) {
                fw.write(t + "," + y.x + "," + y.y + "," + y.z + "\n");
            }
        }
        fw.close();
    }

    public static void main(String[] args) throws Exception {
        bifurcationLogistic(2.5, 4.0, 6000, 1200, 200, 0.123456);
        simulateLorenz(40.0, 0.005, new Vec3(1.0,1.0,1.0),
                       10.0, 28.0, 8.0/3.0, 5.0, 4);
        System.out.println("Done. Wrote CSV files.");
    }
}

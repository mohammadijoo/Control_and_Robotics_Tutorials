// Chapter2_Lesson1.java
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 2, Lesson 1: Rolling Constraints and Instantaneous Motion

Dependencies:
  - EJML (Efficient Java Matrix Library): https://ejml.org/
Compile (example):
  javac -cp ejml-all-0.43.jar Chapter2_Lesson1.java
Run:
  java  -cp .:ejml-all-0.43.jar Chapter2_Lesson1

This program:
- Builds lateral no-slip constraint matrix A for standard wheels
- Checks A * xi = 0 for a sample twist xi = [vx, vy, omega]
- Computes ICR in body frame for omega != 0
- Computes wheel spin rates from rolling constraint
*/

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.ArrayList;
import java.util.List;

public class Chapter2_Lesson1 {

    static class Wheel {
        public double lx, ly, alpha, R;
        public Wheel(double lx, double ly, double alpha, double R) {
            this.lx = lx; this.ly = ly; this.alpha = alpha; this.R = R;
        }
    }

    static double[] tDir(double a) {
        return new double[]{Math.cos(a), Math.sin(a)};
    }

    static double[] nDir(double a) {
        return new double[]{-Math.sin(a), Math.cos(a)};
    }

    static double[] vPoint(double vx, double vy, double omega, double[] r) {
        // v(r) = v + omega * k x r, with k x [x,y] = [-y, x]
        return new double[]{vx + omega * (-r[1]), vy + omega * (r[0])};
    }

    static double[] lateralConstraintRow(Wheel w) {
        double[] r = new double[]{w.lx, w.ly};
        double[] n = nDir(w.alpha);
        double c = (-r[1] * n[0] + r[0] * n[1]);
        return new double[]{n[0], n[1], c};
    }

    static DMatrixRMaj buildA(List&lt;Wheel&gt; wheels) {
        DMatrixRMaj A = new DMatrixRMaj(wheels.size(), 3);
        for (int i = 0; i &lt; wheels.size(); i++) {
            double[] row = lateralConstraintRow(wheels.get(i));
            A.set(i, 0, row[0]);
            A.set(i, 1, row[1]);
            A.set(i, 2, row[2]);
        }
        return A;
    }

    static double[] wheelSpinRates(List&lt;Wheel&gt; wheels, double vx, double vy, double omega) {
        double[] rates = new double[wheels.size()];
        for (int i = 0; i &lt; wheels.size(); i++) {
            Wheel w = wheels.get(i);
            double[] r = new double[]{w.lx, w.ly};
            double[] t = tDir(w.alpha);
            double[] vpt = vPoint(vx, vy, omega, r);
            double dot = t[0]*vpt[0] + t[1]*vpt[1];
            rates[i] = dot / w.R;
        }
        return rates;
    }

    static boolean icrBody(double vx, double vy, double omega, double[] outP) {
        double eps = 1e-12;
        if (Math.abs(omega) &lt; eps) return false;
        outP[0] = -vy / omega;
        outP[1] =  vx / omega;
        return true;
    }

    static double axleLineResidual(Wheel w, double[] p) {
        double[] r = new double[]{w.lx, w.ly};
        double[] n = nDir(w.alpha);
        double dx = p[0] - r[0];
        double dy = p[1] - r[1];
        // 2D cross product (p-r) x n = dx*ny - dy*nx
        return dx*n[1] - dy*n[0];
    }

    public static void main(String[] args) {
        System.out.println("=== Demo: two parallel wheels (constraints + ICR) ===");
        double b = 0.6;
        List&lt;Wheel&gt; wheels = new ArrayList&lt;&gt;();
        wheels.add(new Wheel(0.0, +b/2.0, 0.0, 0.1));
        wheels.add(new Wheel(0.0, -b/2.0, 0.0, 0.1));

        double vx = 0.5, vy = 0.0, omega = 0.8;

        DMatrixRMaj A = buildA(wheels);
        DMatrixRMaj xi = new DMatrixRMaj(3,1,true, new double[]{vx, vy, omega});
        DMatrixRMaj res = new DMatrixRMaj(A.numRows, 1);
        CommonOps_DDRM.mult(A, xi, res);

        System.out.println("A = ");
        A.print();
        System.out.println("A*xi = ");
        res.print();

        double[] p = new double[2];
        if (icrBody(vx, vy, omega, p)) {
            System.out.println("ICR (body) = [" + p[0] + ", " + p[1] + "]");
            for (int i = 0; i &lt; wheels.size(); i++) {
                System.out.println("Axle-line residual wheel " + (i+1) + ": " +
                        axleLineResidual(wheels.get(i), p));
            }
        } else {
            System.out.println("omega ~ 0, ICR at infinity (pure translation).");
        }

        double[] rates = wheelSpinRates(wheels, vx, vy, omega);
        System.out.print("Wheel spin rates [rad/s]: ");
        for (double r : rates) System.out.print(r + " ");
        System.out.println();
    }
}

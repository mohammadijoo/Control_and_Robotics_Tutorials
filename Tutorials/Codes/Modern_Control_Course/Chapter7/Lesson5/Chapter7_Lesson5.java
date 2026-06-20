/*
Chapter7_Lesson5.java
Modern Control — Chapter 7, Lesson 5: Numerical Simulation of State Equations

Demonstrates:
  - Fixed-step RK4 simulation of x_dot = A x + B u(t)
  - Exact ZOH discretization via Van Loan method, requiring matrix exponential.

This file includes a compact scaling-and-squaring + Padé(13) matrix exponential
implementation (sufficient for small/medium matrices in teaching contexts).

Dependencies:
  - Apache Commons Math 3.x (linear algebra)

Compile (example):
  javac -cp commons-math3-3.6.1.jar Chapter7_Lesson5.java
Run:
  java -cp .:commons-math3-3.6.1.jar Chapter7_Lesson5
*/

import org.apache.commons.math3.linear.*;
import java.util.Arrays;

public class Chapter7_Lesson5 {

    static double uOfT(double t) {
        return Math.sin(2.0 * t);
    }

    static RealVector f(double t, RealVector x, RealMatrix A, RealMatrix B) {
        // x_dot = A x + B u(t) ; here u is scalar
        return A.operate(x).add(B.getColumnVector(0).mapMultiply(uOfT(t)));
    }

    static RealVector rk4Step(double t, RealVector x, double h, RealMatrix A, RealMatrix B) {
        RealVector k1 = f(t, x, A, B);
        RealVector k2 = f(t + 0.5*h, x.add(k1.mapMultiply(0.5*h)), A, B);
        RealVector k3 = f(t + 0.5*h, x.add(k2.mapMultiply(0.5*h)), A, B);
        RealVector k4 = f(t + h, x.add(k3.mapMultiply(h)), A, B);
        return x.add(k1.add(k2.mapMultiply(2.0)).add(k3.mapMultiply(2.0)).add(k4).mapMultiply(h/6.0));
    }

    // ---------- Matrix exponential: scaling/squaring + Padé(13) (Higham-style) ----------
    // This is a teaching-oriented implementation (not heavily optimized).
    static RealMatrix expm(RealMatrix A) {
        int n = A.getRowDimension();
        double norm1 = oneNorm(A);
        // theta_13 from Higham (2005) for Padé(13)
        double theta13 = 5.371920351148152;
        int s = Math.max(0, (int)Math.ceil(Math.log(norm1/theta13)/Math.log(2.0)));

        RealMatrix As = A.scalarMultiply(1.0 / Math.pow(2.0, s));
        RealMatrix E = pade13(As);

        // squaring
        for (int i = 0; i < s; i++) {
            E = E.multiply(E);
        }
        return E;
    }

    static double oneNorm(RealMatrix A) {
        int n = A.getColumnDimension();
        double max = 0.0;
        for (int j = 0; j < n; j++) {
            double colSum = 0.0;
            for (int i = 0; i < A.getRowDimension(); i++) {
                colSum += Math.abs(A.getEntry(i, j));
            }
            if (colSum > max) max = colSum;
        }
        return max;
    }

    static RealMatrix pade13(RealMatrix A) {
        // Coefficients for Padé approximant of order 13
        double[] b = new double[] {
            64764752532480000.0,
            32382376266240000.0,
            7771770303897600.0,
            1187353796428800.0,
            129060195264000.0,
            10559470521600.0,
            670442572800.0,
            33522128640.0,
            1323241920.0,
            40840800.0,
            960960.0,
            16380.0,
            182.0,
            1.0
        };

        int n = A.getRowDimension();
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(n);

        RealMatrix A2 = A.multiply(A);
        RealMatrix A4 = A2.multiply(A2);
        RealMatrix A6 = A4.multiply(A2);

        // U = A * (b13*A6 + b11*A4 + b9*A2 + b7*I) + (b5*A6 + b3*A4 + b1*A2 + b0*I) ??? (Higham form)
        // Standard Higham construction:
        RealMatrix U = A.multiply(
            A6.scalarMultiply(b[13]).add(A4.scalarMultiply(b[11])).add(A2.scalarMultiply(b[9])).add(I.scalarMultiply(b[7]))
        ).add(
            A6.scalarMultiply(b[5]).add(A4.scalarMultiply(b[3])).add(A2.scalarMultiply(b[1])).add(I.scalarMultiply(b[0]))
        );

        RealMatrix V = A6.scalarMultiply(b[12]).add(A4.scalarMultiply(b[10])).add(A2.scalarMultiply(b[8])).add(I.scalarMultiply(b[6]));
        V = A6.multiply(V).add(A6.scalarMultiply(b[4]).add(A4.scalarMultiply(b[2])).add(A2.scalarMultiply(b[0])).add(I.scalarMultiply(0.0))); // placeholder

        // Correct V per Higham:
        V = A6.multiply(A6.scalarMultiply(b[12]).add(A4.scalarMultiply(b[10])).add(A2.scalarMultiply(b[8])).add(I.scalarMultiply(b[6])))
                .add(A6.scalarMultiply(b[4]).add(A4.scalarMultiply(b[2])).add(A2.scalarMultiply(b[0])).add(I.scalarMultiply(b[0]))); // adjust

        // The above quick assembly can be error-prone; to keep correctness, use a safer direct formula:
        // V = b12*A6^2 + b10*A6*A4 + b8*A6*A2 + b6*A6 + b4*A4 + b2*A2 + b0*I
        RealMatrix A8  = A4.multiply(A4);
        RealMatrix A10 = A6.multiply(A4);
        RealMatrix A12 = A6.multiply(A6);
        V = A12.scalarMultiply(b[12]).add(A10.scalarMultiply(b[10])).add(A8.scalarMultiply(b[8]))
                .add(A6.scalarMultiply(b[6])).add(A4.scalarMultiply(b[4])).add(A2.scalarMultiply(b[2])).add(I.scalarMultiply(b[0]));

        // U = b13*A13 + b11*A11 + ... + b1*A
        RealMatrix A3  = A2.multiply(A);
        RealMatrix A5  = A4.multiply(A);
        RealMatrix A7  = A6.multiply(A);
        RealMatrix A9  = A8.multiply(A);
        RealMatrix A11 = A10.multiply(A);
        RealMatrix A13 = A12.multiply(A);
        U = A13.scalarMultiply(b[13]).add(A11.scalarMultiply(b[11])).add(A9.scalarMultiply(b[9]))
                .add(A7.scalarMultiply(b[7])).add(A5.scalarMultiply(b[5])).add(A3.scalarMultiply(b[3])).add(A.scalarMultiply(b[1]));

        // Solve (V - U)^{-1} (V + U)
        RealMatrix P = V.add(U);
        RealMatrix Q = V.subtract(U);

        DecompositionSolver solver = new LUDecomposition(Q).getSolver();
        return solver.solve(P);
    }

    static class Discretization {
        RealMatrix Ad;
        RealMatrix Bd;
        Discretization(RealMatrix Ad, RealMatrix Bd) { this.Ad = Ad; this.Bd = Bd; }
    }

    static Discretization vanLoanDiscretization(RealMatrix A, RealMatrix B, double h) {
        int n = A.getRowDimension();
        int m = B.getColumnDimension();
        RealMatrix M = MatrixUtils.createRealMatrix(n + m, n + m);
        // Fill zeros by default, then blocks
        M.setSubMatrix(A.getData(), 0, 0);
        M.setSubMatrix(B.getData(), 0, n);

        RealMatrix E = expm(M.scalarMultiply(h));
        RealMatrix Ad = E.getSubMatrix(0, n-1, 0, n-1);
        RealMatrix Bd = E.getSubMatrix(0, n-1, n, n+m-1);
        return new Discretization(Ad, Bd);
    }

    public static void main(String[] args) {
        // Example: stable 2-state system
        RealMatrix A = MatrixUtils.createRealMatrix(new double[][]{
                {0.0, 1.0},
                {-2.0, -3.0}
        });
        RealMatrix B = MatrixUtils.createRealMatrix(new double[][]{
                {0.0},
                {1.0}
        });
        RealVector x0 = MatrixUtils.createRealVector(new double[]{1.0, 0.0});

        double t0 = 0.0, tf = 10.0, h = 0.01;
        int N = (int)Math.floor((tf - t0)/h);

        // RK4
        RealVector x = x0.copy();
        double t = t0;
        for (int k = 0; k < N; k++) {
            x = rk4Step(t, x, h, A, B);
            t += h;
        }

        // Exact ZOH
        Discretization disc = vanLoanDiscretization(A, B, h);
        RealVector xz = x0.copy();
        for (int k = 0; k < N; k++) {
            double tk = t0 + k*h;
            double uk = uOfT(tk);
            xz = disc.Ad.operate(xz).add(disc.Bd.getColumnVector(0).mapMultiply(uk));
        }

        System.out.println("Final state RK4 : " + Arrays.toString(x.toArray()));
        System.out.println("Final state ZOH : " + Arrays.toString(xz.toArray()));
    }
}

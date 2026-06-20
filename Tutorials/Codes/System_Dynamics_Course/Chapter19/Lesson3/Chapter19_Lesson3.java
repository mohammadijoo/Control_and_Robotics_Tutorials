/*
Chapter19_Lesson3.java
Finite Difference (FD) and Finite Element (FE) Approximations (Conceptual Level)

This Java example shows:
  1) 1D heat equation u_t = alpha u_xx via FD in space and theta-method in time.
  2) 1D Poisson -u'' = f(x) via linear FEM (Galerkin) on uniform mesh.

Recommended linear algebra library:
  - EJML (Efficient Java Matrix Library)
    https://ejml.org/

This code uses EJML's SimpleMatrix for clarity (dense matrices). For large N, use EJML sparse.

Compile/run (example with Maven/Gradle):
  Ensure EJML is on the classpath.

Author: (course material generator)
*/

import org.ejml.simple.SimpleMatrix;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class Chapter19_Lesson3 {

    static class Grid1D {
        double L;
        int N;
        double[] x;
        double dx;

        Grid1D(double L, int N) {
            if (N < 3) throw new IllegalArgumentException("N must be >= 3");
            this.L = L; this.N = N;
            this.x = new double[N];
            for (int i = 0; i < N; i++) x[i] = (L * i) / (N - 1);
            this.dx = x[1] - x[0];
        }
    }

    // Dense Laplacian on interior nodes with Dirichlet boundaries.
    static SimpleMatrix laplacianDirichlet1D(int N, double dx) {
        int n = N - 2;
        SimpleMatrix A = new SimpleMatrix(n, n);
        double s = 1.0 / (dx * dx);
        for (int i = 0; i < n; i++) {
            A.set(i, i, -2.0 * s);
            if (i - 1 >= 0) A.set(i, i - 1, 1.0 * s);
            if (i + 1 < n)  A.set(i, i + 1, 1.0 * s);
        }
        return A;
    }

    static double[] heatFDTheta(
            double[] u0Full,
            double alpha,
            Grid1D grid,
            double dt,
            int steps,
            double theta
    ) {
        int N = grid.N;
        int n = N - 2;

        SimpleMatrix A = laplacianDirichlet1D(N, grid.dx);
        SimpleMatrix I = SimpleMatrix.identity(n);

        SimpleMatrix LHS = I.minus(A.scale(theta * dt * alpha));
        SimpleMatrix RHS = I.plus(A.scale((1.0 - theta) * dt * alpha));

        double uL = u0Full[0];
        double uR = u0Full[N - 1];

        // boundary vector bc for interior equation
        double s = 1.0 / (grid.dx * grid.dx);
        SimpleMatrix bc = new SimpleMatrix(n, 1);
        bc.set(0, 0, uL * s);
        bc.set(n - 1, 0, uR * s);

        SimpleMatrix v = new SimpleMatrix(n, 1);
        for (int i = 0; i < n; i++) v.set(i, 0, u0Full[i + 1]);

        for (int k = 0; k < steps; k++) {
            SimpleMatrix rhs = RHS.mult(v).plus(bc.scale(dt * alpha));
            v = LHS.solve(rhs); // dense solve
        }

        double[] uFinal = new double[N];
        uFinal[0] = uL;
        uFinal[N - 1] = uR;
        for (int i = 0; i < n; i++) uFinal[i + 1] = v.get(i, 0);
        return uFinal;
    }

    static class FemResult {
        double[] x;
        double[] u;
        FemResult(double[] x, double[] u) { this.x = x; this.u = u; }
    }

    // Linear FEM for -u'' = f(x) on (0,L), Dirichlet u(0)=u0, u(L)=uL.
    static FemResult femPoissonP1(int Nel, double L, java.util.function.DoubleUnaryOperator f, double u0, double uL) {
        int Nn = Nel + 1;
        double[] x = new double[Nn];
        double h = L / Nel;
        for (int i = 0; i < Nn; i++) x[i] = i * h;

        SimpleMatrix K = new SimpleMatrix(Nn, Nn);
        SimpleMatrix F = new SimpleMatrix(Nn, 1);

        for (int e = 0; e < Nel; e++) {
            int i = e, j = e + 1;
            SimpleMatrix Ke = new SimpleMatrix(2, 2);
            Ke.set(0,0,  1.0/h); Ke.set(0,1, -1.0/h);
            Ke.set(1,0, -1.0/h); Ke.set(1,1,  1.0/h);

            double xm = 0.5 * (x[i] + x[j]);
            double fe = f.applyAsDouble(xm);
            double load = fe * (h / 2.0);

            // assemble
            K.set(i, i, K.get(i,i) + Ke.get(0,0));
            K.set(i, j, K.get(i,j) + Ke.get(0,1));
            K.set(j, i, K.get(j,i) + Ke.get(1,0));
            K.set(j, j, K.get(j,j) + Ke.get(1,1));

            F.set(i, 0, F.get(i,0) + load);
            F.set(j, 0, F.get(j,0) + load);
        }

        // Dirichlet elimination: solve for interior nodes 1..Nn-2
        int n = Nn - 2;
        SimpleMatrix Kff = new SimpleMatrix(n, n);
        SimpleMatrix Ff  = new SimpleMatrix(n, 1);

        for (int a = 0; a < n; a++) {
            int I = a + 1;
            double rhs = F.get(I, 0) - K.get(I, 0) * u0 - K.get(I, Nn - 1) * uL;
            Ff.set(a, 0, rhs);
            for (int b = 0; b < n; b++) {
                int J = b + 1;
                Kff.set(a, b, K.get(I, J));
            }
        }

        SimpleMatrix uf = Kff.solve(Ff);

        double[] u = new double[Nn];
        u[0] = u0;
        u[Nn - 1] = uL;
        for (int a = 0; a < n; a++) u[a + 1] = uf.get(a, 0);

        return new FemResult(x, u);
    }

    static double fRhs(double x) {
        // -u'' = pi^2 sin(pi x) gives exact u = sin(pi x) on [0,1]
        return Math.PI * Math.PI * Math.sin(Math.PI * x);
    }

    static void writeCSV(String filename, double[] x, double[] u, double[] uex) throws IOException {
        try (FileWriter fw = new FileWriter(filename)) {
            fw.write("x,u,uex\n");
            for (int i = 0; i < x.length; i++) {
                fw.write(String.format(Locale.US, "%.10f,%.10f,%.10f\n", x[i], u[i], uex[i]));
            }
        }
    }

    public static void main(String[] args) throws IOException {
        Grid1D grid = new Grid1D(1.0, 101);
        double alpha = 0.05;

        double[] u0 = new double[grid.N];
        for (int i = 0; i < grid.N; i++) u0[i] = Math.sin(Math.PI * grid.x[i]);
        u0[0] = 0.0; u0[grid.N - 1] = 0.0;

        double dt = 0.4 * (grid.dx * grid.dx) / alpha;
        int steps = 200;
        double[] uHeatFinal = heatFDTheta(u0, alpha, grid, dt, steps, 0.5);

        // write heat solution
        double[] uexHeat = new double[grid.N];
        for (int i = 0; i < grid.N; i++) uexHeat[i] = uHeatFinal[i]; // no analytic in this demo
        writeCSV("heat_fd_final_java.csv", grid.x, uHeatFinal, uexHeat);

        // FEM Poisson
        FemResult fr = femPoissonP1(40, 1.0, Chapter19_Lesson3::fRhs, 0.0, 0.0);
        double[] uex = new double[fr.x.length];
        double errInf = 0.0;
        for (int i = 0; i < fr.x.length; i++) {
            uex[i] = Math.sin(Math.PI * fr.x[i]);
            errInf = Math.max(errInf, Math.abs(fr.u[i] - uex[i]));
        }
        System.out.println("FEM Poisson max error = " + errInf);
        writeCSV("poisson_fem_java.csv", fr.x, fr.u, uex);

        System.out.println("Wrote heat_fd_final_java.csv and poisson_fem_java.csv");
    }
}

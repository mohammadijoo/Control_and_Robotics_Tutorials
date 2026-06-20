// Chapter8_Lesson1.java
// Modern Control — Chapter 8, Lesson 1: Definition of the State Transition Matrix
//
// This example computes Phi(t,t0) ≈ exp(A*(t-t0)) using a truncated power series,
// implemented with Apache Commons Math matrices.
//
// Maven dependency:
//   <dependency>
//     <groupId>org.apache.commons</groupId>
//     <artifactId>commons-math3</artifactId>
//     <version>3.6.1</version>
//   </dependency>
//
// Compile (with commons-math3 on classpath):
//   javac -cp commons-math3-3.6.1.jar Chapter8_Lesson1.java
// Run:
//   java -cp .:commons-math3-3.6.1.jar Chapter8_Lesson1

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class Chapter8_Lesson1 {

    public static RealMatrix expmSeries(RealMatrix A, double tau, int terms) {
        int n = A.getRowDimension();
        RealMatrix M = A.scalarMultiply(tau);
        RealMatrix term = MatrixUtils.createRealIdentityMatrix(n);
        RealMatrix sum  = MatrixUtils.createRealIdentityMatrix(n);

        for (int k = 1; k <= terms; k++) {
            term = term.multiply(M).scalarMultiply(1.0 / ((double) k));
            sum  = sum.add(term);
        }
        return sum;
    }

    public static void main(String[] args) {
        double[][] Adata = new double[][]{
            {0.0,  1.0},
            {-2.0, -3.0}
        };
        RealMatrix A = MatrixUtils.createRealMatrix(Adata);

        double[] x0 = new double[]{1.0, -0.5};

        double t0 = 0.0;
        double t  = 1.25;
        double tau = t - t0;

        int terms = 30;
        RealMatrix Phi = expmSeries(A, tau, terms);

        double[] x = Phi.operate(x0);

        System.out.println("tau = t - t0 = " + tau);
        System.out.println("Phi(t,t0) approx (series, terms=" + terms + "):");
        System.out.println(Phi.toString());
        System.out.println("x(t) = Phi x0:");
        System.out.println("[" + x[0] + ", " + x[1] + "]");

        // Finite-difference derivative check: Phi_dot approx vs A*Phi
        double dt = 1e-6;
        RealMatrix Phi_t   = expmSeries(A, tau, terms);
        RealMatrix Phi_tdt = expmSeries(A, tau + dt, terms);
        RealMatrix Phi_dot_fd = Phi_tdt.subtract(Phi_t).scalarMultiply(1.0 / dt);
        RealMatrix residual = Phi_dot_fd.subtract(A.multiply(Phi_t));
        double fro = residual.getFrobeniusNorm();
        System.out.println("||Phi_dot_fd - A Phi||_F = " + fro);
    }
}

// Chapter19_Lesson2.java
// Separation of Variables & Modal Decomposition (1D Heat Equation, Dirichlet BC)
//
// PDE: u_t = alpha * u_xx,  u(0,t)=u(L,t)=0
// Solution: u(x,t) = sum_{n=1..N} a_n exp(-alpha (n*pi/L)^2 t) sin(n*pi x/L)
//
// This program:
// 1) computes a_n numerically using the trapezoidal rule for u0(x)=x(L-x)
// 2) prints u(x,t) on a grid as CSV (x,u)

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.Locale;

public class Chapter19_Lesson2 {
    static double u0(double x, double L) { return x * (L - x); }
    static double phi(int n, double x, double L) { return Math.sin(n * Math.PI * x / L); }

    static double trapzCoeff(int n, double L, int M) {
        double h = L / (M - 1);
        double sum = 0.0;
        for (int i = 0; i < M; i++) {
            double x = i * h;
            double w = (i == 0 || i == M - 1) ? 0.5 : 1.0;
            sum += w * u0(x, L) * phi(n, x, L);
        }
        double integral = h * sum;
        return (2.0 / L) * integral;
    }

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.US);

        final double L = 1.0;
        final double alpha = 0.02;
        final int N = 40;     // modes
        final int M = 5001;   // integration points
        final int Nx = 200;   // spatial grid points

        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        System.out.print("Enter time t (e.g., 2.5): ");
        double t = Double.parseDouble(br.readLine().trim());

        double[] a = new double[N + 1];
        for (int n = 1; n <= N; n++) {
            a[n] = trapzCoeff(n, L, M);
        }

        System.out.println("x,u");
        for (int i = 0; i < Nx; i++) {
            double x = (L * i) / (Nx - 1);
            double u = 0.0;
            for (int n = 1; n <= N; n++) {
                double lam = (n * Math.PI / L) * (n * Math.PI / L);
                u += a[n] * Math.exp(-alpha * lam * t) * phi(n, x, L);
            }
            System.out.printf("%.8f,%.10f%n", x, u);
        }

        System.err.println("Computed with N=" + N + " modes.");
    }
}

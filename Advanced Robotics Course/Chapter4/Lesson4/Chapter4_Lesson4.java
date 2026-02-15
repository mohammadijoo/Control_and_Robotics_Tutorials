import java.util.Arrays;

public class DirectShootingDoubleIntegrator {

    private final int N;
    private final double T;
    private final double h;

    public DirectShootingDoubleIntegrator(int N, double T) {
        this.N = N;
        this.T = T;
        this.h = T / N;
    }

    // Simulate terminal position for a control sequence u
    private double simulateTerminal(double[] u) {
        double p = 0.0;
        double v = 0.0;
        for (int k = 0; k < N; ++k) {
            double uk = u[k];
            p = p + h * v;
            v = v + h * uk;
        }
        return p;
    }

    // Cost: integral of u^2 plus terminal penalty
    private double cost(double[] u) {
        double J = 0.0;
        for (int k = 0; k < N; ++k) {
            J += h * u[k] * u[k];
        }
        double pT = simulateTerminal(u);
        double terminalPenalty = 1000.0 * (pT - 1.0) * (pT - 1.0);
        return J + terminalPenalty;
    }

    // Very simple gradient descent with finite differences
    public double[] gradientDescent(int maxIter, double alpha) {
        double[] u = new double[N];
        Arrays.fill(u, 0.0);
        double eps = 1e-6;

        for (int it = 0; it < maxIter; ++it) {
            double J = cost(u);
            double[] grad = new double[N];

            for (int k = 0; k < N; ++k) {
                double delta = 1e-5;
                double old = u[k];

                u[k] = old + delta;
                double Jp = cost(u);

                u[k] = old - delta;
                double Jm = cost(u);

                u[k] = old;
                grad[k] = (Jp - Jm) / (2.0 * delta);
            }

            double norm = 0.0;
            for (int k = 0; k < N; ++k) {
                norm += grad[k] * grad[k];
            }
            norm = Math.sqrt(norm);
            if (norm < eps) {
                break;
            }

            for (int k = 0; k < N; ++k) {
                u[k] -= alpha * grad[k];
            }
            System.out.println("iter " + it + " cost " + J);
        }
        return u;
    }

    public static void main(String[] args) {
        DirectShootingDoubleIntegrator solver =
            new DirectShootingDoubleIntegrator(40, 1.0);
        double[] u = solver.gradientDescent(200, 1e-2);
        System.out.println("First few controls:");
        for (int k = 0; k < 5; ++k) {
            System.out.println(u[k]);
        }
    }
}
      

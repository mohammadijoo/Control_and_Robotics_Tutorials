// Chapter20_Lesson3.java
// System Dynamics (Control Engineering) - Chapter 20, Lesson 3
// Sensitivity to Initial Conditions and Lyapunov Exponents (Intro)

import java.util.Random;

public class Chapter20_Lesson3 {

    // -----------------------------
    // Part A) Logistic map LLE
    // -----------------------------
    static double logisticStep(double x, double r) {
        return r * x * (1.0 - x);
    }

    static double lyapunovLogistic(double r, double x0, int n, int discard) {
        double x = x0;
        for (int i = 0; i < discard; i++) x = logisticStep(x, r);

        double s = 0.0;
        for (int i = 0; i < n; i++) {
            double fp = r * (1.0 - 2.0 * x);
            s += Math.log(Math.abs(fp) + 1e-300);
            x = logisticStep(x, r);
        }
        return s / n;
    }

    // -----------------------------
    // Part B) Lorenz LLE via two-trajectory renormalization with RK4
    // -----------------------------
    static double[] lorenzRhs(double[] X, double sigma, double rho, double beta) {
        double x = X[0], y = X[1], z = X[2];
        return new double[] {
            sigma * (y - x),
            x * (rho - z) - y,
            x * y - beta * z
        };
    }

    static double[] rk4StepLorenz(double[] X, double dt, double sigma, double rho, double beta) {
        double[] k1 = lorenzRhs(X, sigma, rho, beta);

        double[] X2 = new double[] { X[0] + 0.5*dt*k1[0], X[1] + 0.5*dt*k1[1], X[2] + 0.5*dt*k1[2] };
        double[] k2 = lorenzRhs(X2, sigma, rho, beta);

        double[] X3 = new double[] { X[0] + 0.5*dt*k2[0], X[1] + 0.5*dt*k2[1], X[2] + 0.5*dt*k2[2] };
        double[] k3 = lorenzRhs(X3, sigma, rho, beta);

        double[] X4 = new double[] { X[0] + dt*k3[0], X[1] + dt*k3[1], X[2] + dt*k3[2] };
        double[] k4 = lorenzRhs(X4, sigma, rho, beta);

        return new double[] {
            X[0] + (dt/6.0)*(k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]),
            X[1] + (dt/6.0)*(k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]),
            X[2] + (dt/6.0)*(k1[2] + 2.0*k2[2] + 2.0*k3[2] + k4[2])
        };
    }

    static double norm(double[] v) {
        return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }

    static double lyapunovLorenzLLE(
        double[] X0,
        double dt,
        double T,
        double transient,
        int renormEvery,
        double d0,
        double sigma,
        double rho,
        double beta,
        long seed
    ) {
        Random rng = new Random(seed);
        double[] X = new double[] { X0[0], X0[1], X0[2] };

        // random unit direction
        double[] u = new double[] { rng.nextGaussian(), rng.nextGaussian(), rng.nextGaussian() };
        double un = norm(u);
        u[0] /= un; u[1] /= un; u[2] /= un;

        double[] Xp = new double[] { X[0] + d0*u[0], X[1] + d0*u[1], X[2] + d0*u[2] };

        // transient integration (keep separation ~ d0 but don't accumulate)
        int nTrans = (int)Math.round(transient / dt);
        for (int i = 0; i < nTrans; i++) {
            X  = rk4StepLorenz(X,  dt, sigma, rho, beta);
            Xp = rk4StepLorenz(Xp, dt, sigma, rho, beta);

            double[] dvec = new double[] { Xp[0]-X[0], Xp[1]-X[1], Xp[2]-X[2] };
            double d = norm(dvec);
            if (d == 0.0) {
                Xp[0] = X[0] + d0*u[0]; Xp[1] = X[1] + d0*u[1]; Xp[2] = X[2] + d0*u[2];
            } else {
                double s = d0 / d;
                Xp[0] = X[0] + s*dvec[0]; Xp[1] = X[1] + s*dvec[1]; Xp[2] = X[2] + s*dvec[2];
            }
        }

        // main accumulation
        double sum = 0.0;
        int steps = (int)Math.round(T / dt);
        int count = 0;
        for (int k = 0; k < steps; k++) {
            X  = rk4StepLorenz(X,  dt, sigma, rho, beta);
            Xp = rk4StepLorenz(Xp, dt, sigma, rho, beta);

            if ((k+1) % renormEvery == 0) {
                double[] dvec = new double[] { Xp[0]-X[0], Xp[1]-X[1], Xp[2]-X[2] };
                double d = norm(dvec);
                if (d == 0.0) continue;
                sum += Math.log(d / d0);
                count += 1;

                double s = d0 / d;
                Xp[0] = X[0] + s*dvec[0]; Xp[1] = X[1] + s*dvec[1]; Xp[2] = X[2] + s*dvec[2];
            }
        }

        double totalTime = count * renormEvery * dt;
        return sum / totalTime;
    }

    public static void main(String[] args) {
        System.out.println("Logistic map LLE examples:");
        double[] rList = new double[] {3.2, 3.5, 3.9, 4.0};
        for (double r : rList) {
            double lam = lyapunovLogistic(r, 0.234, 100000, 5000);
            System.out.printf("  r=%.1f : lambda ~= %.6f%n", r, lam);
        }

        System.out.println("\nLorenz LLE example (sigma=10, rho=28, beta=8/3):");
        double lamL = lyapunovLorenzLLE(new double[]{1,1,1}, 0.01, 120.0, 20.0, 10, 1e-8, 10.0, 28.0, 8.0/3.0, 0);
        System.out.printf("  lambda_max ~= %.4f 1/time-unit%n", lamL);
    }
}

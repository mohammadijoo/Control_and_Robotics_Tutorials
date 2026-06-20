/*
Chapter28_Lesson5.java

Analytic continuous-time LQR preview for the double integrator.

No external library is required. The program uses the special closed-form
CARE solution for:
    A = [[0, 1], [0, 0]]
    B = [[0], [1]]
    Q = diag(q1, q2)
    R = r
*/

public class Chapter28_Lesson5 {
    static double[] dx(double[][] Acl, double[] x) {
        return new double[] {
            Acl[0][0] * x[0] + Acl[0][1] * x[1],
            Acl[1][0] * x[0] + Acl[1][1] * x[1]
        };
    }

    static double[] rk4(double[][] Acl, double[] x, double h) {
        double[] k1 = dx(Acl, x);
        double[] x2 = {x[0] + 0.5 * h * k1[0], x[1] + 0.5 * h * k1[1]};
        double[] k2 = dx(Acl, x2);
        double[] x3 = {x[0] + 0.5 * h * k2[0], x[1] + 0.5 * h * k2[1]};
        double[] k3 = dx(Acl, x3);
        double[] x4 = {x[0] + h * k3[0], x[1] + h * k3[1]};
        double[] k4 = dx(Acl, x4);

        return new double[] {
            x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        };
    }

    public static void main(String[] args) {
        double q1 = 10.0;
        double q2 = 1.0;
        double r = 0.2;

        // CARE for double integrator:
        // p12 = sqrt(q1*r)
        // p22 = sqrt(r*(2*p12 + q2))
        // p11 = p12*p22/r
        double p12 = Math.sqrt(q1 * r);
        double p22 = Math.sqrt(r * (2.0 * p12 + q2));
        double p11 = p12 * p22 / r;

        double k1 = p12 / r;
        double k2 = p22 / r;

        double[][] Acl = {
            {0.0, 1.0},
            {-k1, -k2}
        };

        double[] x = {1.0, 0.0};
        double h = 0.002;
        int steps = (int)(8.0 / h);
        double J = 0.0;

        for (int i = 0; i < steps; i++) {
            double u = -(k1 * x[0] + k2 * x[1]);
            double integrand = q1 * x[0] * x[0] + q2 * x[1] * x[1] + r * u * u;
            J += integrand * h;
            x = rk4(Acl, x, h);
        }

        System.out.printf("P = [[%.8f, %.8f], [%.8f, %.8f]]%n", p11, p12, p12, p22);
        System.out.printf("K = [%.8f, %.8f]%n", k1, k2);
        System.out.printf("finite-horizon estimated cost over [0, 8] = %.8f%n", J);
        System.out.printf("infinite-horizon value x0^T P x0 = %.8f%n", p11);
    }
}

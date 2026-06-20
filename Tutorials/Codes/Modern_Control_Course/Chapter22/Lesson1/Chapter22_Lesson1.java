// Chapter22_Lesson1.java
// Structure of state-feedback law: u = -K x + r
// Compile: javac Chapter22_Lesson1.java
// Run:     java Chapter22_Lesson1

public class Chapter22_Lesson1 {
    static double[] add(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1]};
    }

    static double[] scale(double s, double[] a) {
        return new double[] {s * a[0], s * a[1]};
    }

    static double[] matVec(double[][] M, double[] x) {
        return new double[] {
            M[0][0] * x[0] + M[0][1] * x[1],
            M[1][0] * x[0] + M[1][1] * x[1]
        };
    }

    static double[] dynamics(double[][] A, double[] B, double[] K, double r, double[] x) {
        double u = -K[0] * x[0] - K[1] * x[1] + r;
        double[] Ax = matVec(A, x);
        return new double[] {Ax[0] + B[0] * u, Ax[1] + B[1] * u};
    }

    static double[] rk4Step(double[][] A, double[] B, double[] K, double r, double h, double[] x) {
        double[] k1 = dynamics(A, B, K, r, x);
        double[] k2 = dynamics(A, B, K, r, add(x, scale(0.5 * h, k1)));
        double[] k3 = dynamics(A, B, K, r, add(x, scale(0.5 * h, k2)));
        double[] k4 = dynamics(A, B, K, r, add(x, scale(h, k3)));

        return add(x, scale(h / 6.0, add(add(k1, scale(2.0, k2)), add(scale(2.0, k3), k4))));
    }

    public static void main(String[] args) {
        double[][] A = {
            {0.0, 1.0},
            {-2.0, -0.4}
        };
        double[] B = {0.0, 1.0};
        double[] C = {1.0, 0.0};
        double[] K = {4.0, 2.6};
        double r = 1.0;

        double[][] Acl = new double[2][2];
        for (int i = 0; i < 2; i++) {
            Acl[i][0] = A[i][0] - B[i] * K[0];
            Acl[i][1] = A[i][1] - B[i] * K[1];
        }

        System.out.println("A - B K:");
        System.out.println(Acl[0][0] + " " + Acl[0][1]);
        System.out.println(Acl[1][0] + " " + Acl[1][1]);

        double trace = Acl[0][0] + Acl[1][1];
        double determinant = Acl[0][0] * Acl[1][1] - Acl[0][1] * Acl[1][0];
        double discriminant = trace * trace - 4.0 * determinant;
        System.out.println("trace = " + trace + ", determinant = " + determinant);
        if (discriminant >= 0.0) {
            System.out.println("eigenvalues = "
                + 0.5 * (trace + Math.sqrt(discriminant)) + ", "
                + 0.5 * (trace - Math.sqrt(discriminant)));
        } else {
            System.out.println("eigenvalues = " + 0.5 * trace
                + " +/- " + 0.5 * Math.sqrt(-discriminant) + " i");
        }

        double[] x = {0.2, 0.0};
        double h = 0.01;
        double tf = 8.0;

        System.out.println("t,x1,x2,u,y");
        for (int step = 0; step <= (int)(tf / h); step++) {
            double t = step * h;
            double u = -K[0] * x[0] - K[1] * x[1] + r;
            double y = C[0] * x[0] + C[1] * x[1];

            if (step % 100 == 0) {
                System.out.println(t + "," + x[0] + "," + x[1] + "," + u + "," + y);
            }

            x = rk4Step(A, B, K, r, h, x);
        }
    }
}

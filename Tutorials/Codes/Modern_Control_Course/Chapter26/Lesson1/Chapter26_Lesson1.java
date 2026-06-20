/*
Chapter26_Lesson1.java

Need for steady-state accuracy in a state-space framework.
Scratch 2-state implementation without external dependencies.

Compile:
  javac Chapter26_Lesson1.java

Run:
  java Chapter26_Lesson1
*/

public class Chapter26_Lesson1 {
    static class Mat2 {
        double a11, a12, a21, a22;

        Mat2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class Vec2 {
        double x1, x2;

        Vec2(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static Mat2 subtractBK(Mat2 A, Vec2 B, Vec2 K) {
        return new Mat2(
            A.a11 - B.x1 * K.x1, A.a12 - B.x1 * K.x2,
            A.a21 - B.x2 * K.x1, A.a22 - B.x2 * K.x2
        );
    }

    static Mat2 inverse2(Mat2 M) {
        double det = M.a11 * M.a22 - M.a12 * M.a21;
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Matrix is singular or nearly singular.");
        }
        return new Mat2(M.a22 / det, -M.a12 / det, -M.a21 / det, M.a11 / det);
    }

    static Vec2 matVec(Mat2 M, Vec2 v) {
        return new Vec2(M.a11 * v.x1 + M.a12 * v.x2,
                        M.a21 * v.x1 + M.a22 * v.x2);
    }

    static double dot(Vec2 c, Vec2 x) {
        return c.x1 * x.x1 + c.x2 * x.x2;
    }

    static Vec2 rhs(Mat2 Acl, Vec2 B, Vec2 x, double inputTotal) {
        Vec2 ax = matVec(Acl, x);
        return new Vec2(ax.x1 + B.x1 * inputTotal,
                        ax.x2 + B.x2 * inputTotal);
    }

    static Vec2 rk4Step(Mat2 Acl, Vec2 B, Vec2 x, double inputTotal, double h) {
        Vec2 k1 = rhs(Acl, B, x, inputTotal);
        Vec2 x2 = new Vec2(x.x1 + 0.5 * h * k1.x1, x.x2 + 0.5 * h * k1.x2);
        Vec2 k2 = rhs(Acl, B, x2, inputTotal);
        Vec2 x3 = new Vec2(x.x1 + 0.5 * h * k2.x1, x.x2 + 0.5 * h * k2.x2);
        Vec2 k3 = rhs(Acl, B, x3, inputTotal);
        Vec2 x4 = new Vec2(x.x1 + h * k3.x1, x.x2 + h * k3.x2);
        Vec2 k4 = rhs(Acl, B, x4, inputTotal);

        return new Vec2(
            x.x1 + (h / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1),
            x.x2 + (h / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2)
        );
    }

    static double simulateFinalY(Mat2 Acl, Vec2 B, Vec2 C,
                                 double nbar, double reference, double disturbance) {
        Vec2 x = new Vec2(0.0, 0.0);
        double h = 0.001;
        double tFinal = 8.0;
        int steps = (int) (tFinal / h);
        double inputTotal = nbar * reference + disturbance;

        for (int i = 0; i < steps; i++) {
            x = rk4Step(Acl, B, x, inputTotal, h);
        }
        return dot(C, x);
    }

    public static void main(String[] args) {
        Mat2 A = new Mat2(0.0, 1.0, -2.0, -3.0);
        Vec2 B = new Vec2(0.0, 1.0);
        Vec2 C = new Vec2(1.0, 0.0);
        Vec2 K = new Vec2(4.0, 2.0);

        Mat2 Acl = subtractBK(A, B, K);
        Mat2 invAcl = inverse2(Acl);
        Vec2 invAclB = matVec(invAcl, B);

        double dcClosed = -dot(C, invAclB);
        double nbar = 1.0 / dcClosed;

        System.out.printf("Closed-loop DC map from prefilter input v to y: %.6f%n", dcClosed);
        System.out.printf("Required static prefilter Nbar: %.6f%n%n", nbar);

        String[] labels = {
            "Nbar = 1, no disturbance",
            "Nbar = computed, no disturbance",
            "Nbar = computed, disturbance d = 0.2"
        };
        double[] nbars = {1.0, nbar, nbar};
        double[] references = {1.0, 1.0, 1.0};
        double[] disturbances = {0.0, 0.0, 0.2};

        for (int i = 0; i < labels.length; i++) {
            double yFinal = simulateFinalY(Acl, B, C, nbars[i], references[i], disturbances[i]);
            System.out.printf("%s: final y approximately %.6f, final error %.6f%n",
                              labels[i], yFinal, references[i] - yFinal);
        }
    }
}

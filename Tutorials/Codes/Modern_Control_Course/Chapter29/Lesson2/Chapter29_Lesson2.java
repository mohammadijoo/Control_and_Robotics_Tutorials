// Chapter29_Lesson2.java
// From-scratch 2x2 implementation for the LTV transition matrix.
// Compile and run:
//   javac Chapter29_Lesson2.java
//   java Chapter29_Lesson2

public class Chapter29_Lesson2 {
    static final class Mat2 {
        final double a00, a01, a10, a11;

        Mat2(double a00, double a01, double a10, double a11) {
            this.a00 = a00;
            this.a01 = a01;
            this.a10 = a10;
            this.a11 = a11;
        }

        static Mat2 zero() {
            return new Mat2(0.0, 0.0, 0.0, 0.0);
        }

        static Mat2 eye() {
            return new Mat2(1.0, 0.0, 0.0, 1.0);
        }

        Mat2 add(Mat2 b) {
            return new Mat2(a00 + b.a00, a01 + b.a01, a10 + b.a10, a11 + b.a11);
        }

        Mat2 sub(Mat2 b) {
            return new Mat2(a00 - b.a00, a01 - b.a01, a10 - b.a10, a11 - b.a11);
        }

        Mat2 scale(double c) {
            return new Mat2(c * a00, c * a01, c * a10, c * a11);
        }

        Mat2 multiply(Mat2 b) {
            return new Mat2(
                a00 * b.a00 + a01 * b.a10,
                a00 * b.a01 + a01 * b.a11,
                a10 * b.a00 + a11 * b.a10,
                a10 * b.a01 + a11 * b.a11
            );
        }

        double normF() {
            return Math.sqrt(a00 * a00 + a01 * a01 + a10 * a10 + a11 * a11);
        }

        void print() {
            System.out.printf("[%12.8f %12.8f]%n", a00, a01);
            System.out.printf("[%12.8f %12.8f]%n", a10, a11);
        }
    }

    static Mat2 A(double t) {
        return new Mat2(
            0.0, 1.0,
            -2.0 - 0.5 * Math.sin(t), -0.4 + 0.2 * Math.cos(2.0 * t)
        );
    }

    static Mat2 rhs(double t, Mat2 phi) {
        return A(t).multiply(phi);
    }

    static Mat2 rk4Phi(double t0, double tf, int steps) {
        double h = (tf - t0) / steps;
        double t = t0;
        Mat2 phi = Mat2.eye();

        for (int i = 0; i < steps; i++) {
            Mat2 k1 = rhs(t, phi);
            Mat2 k2 = rhs(t + 0.5 * h, phi.add(k1.scale(0.5 * h)));
            Mat2 k3 = rhs(t + 0.5 * h, phi.add(k2.scale(0.5 * h)));
            Mat2 k4 = rhs(t + h, phi.add(k3.scale(h)));

            Mat2 incr = k1.add(k2.scale(2.0)).add(k3.scale(2.0)).add(k4).scale(h / 6.0);
            phi = phi.add(incr);
            t += h;
        }
        return phi;
    }

    static Mat2 peanoBaker(double t0, double tf, int order, int steps) {
        double h = (tf - t0) / steps;
        Mat2[][] terms = new Mat2[order + 1][steps + 1];

        for (int k = 0; k <= order; k++) {
            for (int j = 0; j <= steps; j++) {
                terms[k][j] = Mat2.zero();
            }
        }
        for (int j = 0; j <= steps; j++) terms[0][j] = Mat2.eye();

        for (int k = 1; k <= order; k++) {
            terms[k][0] = Mat2.zero();
            for (int j = 1; j <= steps; j++) {
                double smid = t0 + (j - 0.5) * h;
                Mat2 integrand = A(smid).multiply(terms[k - 1][j - 1]);
                terms[k][j] = terms[k][j - 1].add(integrand.scale(h));
            }
        }

        Mat2 phi = Mat2.zero();
        for (int k = 0; k <= order; k++) phi = phi.add(terms[k][steps]);
        return phi;
    }

    public static void main(String[] args) {
        double t0 = 0.0;
        double tf = 6.0;

        Mat2 phiRK = rk4Phi(t0, tf, 20000);
        System.out.println("Phi(tf,t0) by RK4 matrix IVP:");
        phiRK.print();

        int[] orders = {1, 2, 3, 4, 6, 8};
        for (int order : orders) {
            Mat2 phiPB = peanoBaker(t0, tf, order, 6000);
            System.out.printf("Peano-Baker order %d Frobenius error = %.8e%n",
                              order, phiPB.sub(phiRK).normF());
        }

        Mat2 phi60 = rk4Phi(0.0, 6.0, 20000);
        Mat2 phi62 = rk4Phi(2.5, 6.0, 12000);
        Mat2 phi20 = rk4Phi(0.0, 2.5, 8000);
        System.out.printf("Composition error = %.8e%n", phi60.sub(phi62.multiply(phi20)).normF());
    }
}

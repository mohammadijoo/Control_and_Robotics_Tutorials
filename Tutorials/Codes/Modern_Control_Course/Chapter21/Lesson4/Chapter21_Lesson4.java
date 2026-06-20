/*
Chapter21_Lesson4.java
Scratch 2-state SISO transmission-zero classifier.

Compile and run:
  javac Chapter21_Lesson4.java
  java Chapter21_Lesson4
*/

public class Chapter21_Lesson4 {
    static class Complex {
        final double re;
        final double im;
        Complex(double re, double im) { this.re = re; this.im = im; }
        public String toString() {
            if (Math.abs(im) < 1e-12) return String.format("%.6f", re);
            return String.format("%.6f%+.6fi", re, im);
        }
    }

    static class StateSpace2 {
        double a11, a12, a21, a22, b1, b2, c1, c2, d;
        StateSpace2(double a11, double a12, double a21, double a22,
                    double b1, double b2, double c1, double c2, double d) {
            this.a11 = a11; this.a12 = a12; this.a21 = a21; this.a22 = a22;
            this.b1 = b1; this.b2 = b2; this.c1 = c1; this.c2 = c2; this.d = d;
        }
    }

    static Complex[] rootsQuadraticOrLinear(double q2, double q1, double q0) {
        double eps = 1e-12;
        if (Math.abs(q2) < eps) {
            if (Math.abs(q1) < eps) return new Complex[]{};
            return new Complex[]{ new Complex(-q0 / q1, 0.0) };
        }
        double disc = q1 * q1 - 4.0 * q2 * q0;
        if (disc >= 0.0) {
            double sd = Math.sqrt(disc);
            return new Complex[]{ new Complex((-q1 + sd) / (2.0 * q2), 0.0),
                                  new Complex((-q1 - sd) / (2.0 * q2), 0.0) };
        }
        double sd = Math.sqrt(-disc);
        return new Complex[]{ new Complex(-q1 / (2.0 * q2),  sd / (2.0 * q2)),
                              new Complex(-q1 / (2.0 * q2), -sd / (2.0 * q2)) };
    }

    static Complex[] transmissionZeros2State(StateSpace2 s) {
        // q(s) = C adj(sI - A) B + D det(sI - A)
        double q2 = s.d;
        double q1 = s.c1 * s.b1 + s.c2 * s.b2 - s.d * (s.a11 + s.a22);
        double q0 = s.c1 * (-s.a22 * s.b1 + s.a12 * s.b2)
                  + s.c2 * ( s.a21 * s.b1 - s.a11 * s.b2)
                  + s.d * (s.a11 * s.a22 - s.a12 * s.a21);
        return rootsQuadraticOrLinear(q2, q1, q0);
    }

    static String classifyCT(Complex[] zeros) {
        double tol = 1e-9;
        if (zeros.length == 0) return "minimum-phase: no finite zeros detected";
        boolean rhp = false;
        boolean axis = false;
        for (Complex z : zeros) {
            if (z.re > tol) rhp = true;
            if (Math.abs(z.re) <= tol) axis = true;
        }
        if (rhp) return "non-minimum-phase: right-half-plane zero exists";
        if (axis) return "borderline non-minimum-phase: imaginary-axis zero exists";
        return "minimum-phase: all finite zeros are in the open left-half-plane";
    }

    static void analyze(String name, StateSpace2 sys) {
        Complex[] zeros = transmissionZeros2State(sys);
        System.out.println("\n" + name);
        System.out.print("Transmission zeros: ");
        if (zeros.length == 0) System.out.print("none");
        for (Complex z : zeros) System.out.print(z + " ");
        System.out.println("\n" + classifyCT(zeros));
    }

    public static void main(String[] args) {
        StateSpace2 minphase = new StateSpace2(0.0, 1.0, -6.0, -5.0, 0.0, 1.0, 1.0, 1.0, 0.0);
        StateSpace2 nonmin   = new StateSpace2(0.0, 1.0, -6.0, -5.0, 0.0, 1.0, 1.0,-1.0, 0.0);

        analyze("Minimum-phase example: G(s)=(s+1)/((s+2)(s+3))", minphase);
        analyze("Non-minimum-phase example: G(s)=(-s+1)/((s+2)(s+3))", nonmin);
    }
}

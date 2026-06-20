// Chapter25_Lesson2.java
// Role of transmission zeros in state-feedback design limits.
//
// Compile and run:
//   javac Chapter25_Lesson2.java
//   java Chapter25_Lesson2
//
// This file demonstrates the SISO plant
//       G(s) = (1 - s)/(s^2 + 3s + 2)
// and shows that state feedback changes closed-loop poles but not the
// transmission zero at s = +1.

public class Chapter25_Lesson2 {
    static class Complex {
        final double re;
        final double im;

        Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        @Override
        public String toString() {
            if (Math.abs(im) < 1e-12) {
                return String.format("%.6f", re);
            }
            return String.format("%.6f%+.6fi", re, im);
        }
    }

    static Complex[] rootsOfQuadratic(double a, double b, double c) {
        double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            double sqrtDisc = Math.sqrt(disc);
            return new Complex[] {
                new Complex((-b + sqrtDisc) / (2.0 * a), 0.0),
                new Complex((-b - sqrtDisc) / (2.0 * a), 0.0)
            };
        }

        double sqrtDisc = Math.sqrt(-disc);
        return new Complex[] {
            new Complex(-b / (2.0 * a), sqrtDisc / (2.0 * a)),
            new Complex(-b / (2.0 * a), -sqrtDisc / (2.0 * a))
        };
    }

    public static void main(String[] args) {
        double openA1 = 3.0;
        double openA0 = 2.0;

        // Numerator 1 - s = -s + 1.
        double numS = -1.0;
        double num0 = 1.0;
        double zero = -num0 / numS;

        Complex[] openPoles = rootsOfQuadratic(1.0, openA1, openA0);

        System.out.println("Open-loop G(s) = (1 - s)/(s^2 + 3s + 2)");
        System.out.println("Transmission zero: " + String.format("%.6f", zero));
        System.out.println("Open-loop poles: " + openPoles[0] + ", " + openPoles[1]);
        System.out.println();

        // Desired closed-loop poles: -5 and -6.
        double desiredA1 = 11.0;
        double desiredA0 = 30.0;
        double k1 = desiredA0 - openA0;
        double k2 = desiredA1 - openA1;

        Complex[] closedPoles = rootsOfQuadratic(1.0, desiredA1, desiredA0);

        System.out.println("State-feedback gain K = [" + k1 + ", " + k2 + "]");
        System.out.println("Closed-loop denominator: s^2 + " + desiredA1 + "s + " + desiredA0);
        System.out.println("Closed-loop poles: " + closedPoles[0] + ", " + closedPoles[1]);
        System.out.println("Closed-loop numerator remains: 1 - s");
        System.out.println("Closed-loop transmission zero remains: " + String.format("%.6f", zero));
        System.out.println();

        double numeratorAtZero = 1.0 - zero;
        System.out.println("Numerator evaluated at s = +1: " + String.format("%.6f", numeratorAtZero));
        System.out.println("Conclusion: pole assignment changes modes, not transmission zeros.");
        System.out.println("A right-half-plane zero imposes tracking and bandwidth limits even after fast pole placement.");
    }
}

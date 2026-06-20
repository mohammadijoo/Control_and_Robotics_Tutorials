// Chapter28_Lesson4.java
// Scalar closed-form LQR illustration for qualitative Q/R effects.
// Compile and run:
//   javac Chapter28_Lesson4.java
//   java Chapter28_Lesson4

public class Chapter28_Lesson4 {
    static class CaseData {
        String name;
        double q;
        double r;

        CaseData(String name, double q, double r) {
            this.name = name;
            this.q = q;
            this.r = r;
        }
    }

    public static void main(String[] args) {
        // Scalar plant: x_dot = a x + b u, u = -k x.
        // Stabilizing scalar LQR gain:
        // k = (a + sqrt(a^2 + b^2 q/r)) / b, for b > 0.
        double a = 0.4;
        double b = 1.0;
        double x0 = 1.0;

        CaseData[] cases = {
            new CaseData("balanced", 1.0, 1.0),
            new CaseData("larger_state_weight", 25.0, 1.0),
            new CaseData("larger_input_weight", 1.0, 25.0),
            new CaseData("small_input_weight", 1.0, 0.04)
        };

        System.out.println("case, q, r, k, closed_loop_lambda, settling_indicator, initial_u");

        for (CaseData c : cases) {
            double k = (a + Math.sqrt(a * a + b * b * c.q / c.r)) / b;
            double lambdaCl = a - b * k;
            double settlingIndicator = -1.0 / lambdaCl;
            double u0 = -k * x0;

            System.out.printf(
                "%s, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f%n",
                c.name, c.q, c.r, k, lambdaCl, settlingIndicator, u0
            );
        }
    }
}

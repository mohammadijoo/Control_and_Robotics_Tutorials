
import java.util.Random;

public class Workspace2R {
    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.7;
        double q1Min = -Math.PI, q1Max = Math.PI;
        double q2Min = -Math.PI, q2Max = Math.PI;
        int N = 10000;

        Random rng = new Random(0);

        for (int k = 0; k < 10; k++) {
            double q1 = q1Min + (q1Max - q1Min) * rng.nextDouble();
            double q2 = q2Min + (q2Max - q2Min) * rng.nextDouble();

            double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
            double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);

            System.out.println(x + " " + y);
        }
        // For full visualization, store all points and plot using a Java plotting library.
    }
}
      
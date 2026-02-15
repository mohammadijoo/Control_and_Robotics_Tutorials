
import java.util.Random;

public class SenseThinkActDemo {
    public static void main(String[] args) {
        double a = 1.02, b = 0.05, K = 1.5;
        int N = 2000;
        Random rng = new Random(0);

        double sigmaW = 0.02, sigmaV = 0.05;
        double x = 0.0;

        for (int k = 0; k < N; k++) {
            double r = 1.0;
            double v = rng.nextGaussian() * sigmaV;
            double y = x + v;           // Sense
            double u = -K * y + r;      // Think
            double w = rng.nextGaussian() * sigmaW;
            x = a * x + b * u + w;      // Act/Plant
        }
        System.out.println("Final state: " + x);
    }
}
      
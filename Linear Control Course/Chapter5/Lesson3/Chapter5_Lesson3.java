import java.util.Locale;

public class FirstOrderStep {
    public static void main(String[] args) {
        double K = 1.0;
        double tau = 0.4;
        double tEnd = 5.0 * tau;
        int N = 200;
        double dt = tEnd / (N - 1);

        Locale.setDefault(Locale.US);
        for (int k = 0; k != N; ++k) {
            double t = k * dt;
            double y = K * (1.0 - Math.exp(-t / tau));
            System.out.printf("%.4f,%.4f%n", t, y);
        }
    }
}

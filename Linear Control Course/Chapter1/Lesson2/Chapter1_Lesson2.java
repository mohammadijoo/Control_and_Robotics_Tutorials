public class FirstOrderFeedbackExample {
    public static void main(String[] args) {
        double a = 1.0;
        double b = 1.0;
        double K = 2.0;
        double h = 0.01;
        int    N = 1000;
        double r = 1.0;

        double[] yOpen = new double[N + 1];
        double[] yClosed = new double[N + 1];

        double uOpen = a / b * r;

        for (int k = 0; k < N; ++k) {
            // Open-loop update
            yOpen[k + 1] = yOpen[k] +
                    h * (-a * yOpen[k] + b * uOpen);

            // Closed-loop update
            double eK = r - yClosed[k];
            double uClosed = K * eK;
            yClosed[k + 1] = yClosed[k] +
                    h * (-a * yClosed[k] + b * uClosed);
        }

        System.out.println("k,time,yOpen,yClosed");
        for (int k = 0; k <= N; ++k) {
            double t = k * h;
            System.out.println(
                k + "," + t + "," + yOpen[k] + "," + yClosed[k]
            );
        }
    }
}

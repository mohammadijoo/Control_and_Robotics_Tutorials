public class GraspScoring {

    // Sigmoid function
    private static double sigmoid(double z) {
        return 1.0 / (1.0 + Math.exp(-z));
    }

    // Train 1D logistic regression with gradient descent
    public static double[] trainLogistic(double[] q, int[] y,
                                         double lr, int epochs) {
        double w = 0.0;
        double b = 0.0;
        int n = q.length;
        for (int e = 0; e < epochs; ++e) {
            double gradW = 0.0;
            double gradB = 0.0;
            for (int i = 0; i < n; ++i) {
                double z = w * q[i] + b;
                double p = sigmoid(z);
                gradW += -(y[i] - p) * q[i];
                gradB += -(y[i] - p);
            }
            gradW /= n;
            gradB /= n;
            w -= lr * gradW;
            b -= lr * gradB;
        }
        return new double[]{w, b};
    }

    public static double predict(double q, double w, double b) {
        return sigmoid(w * q + b);
    }

    public static void main(String[] args) {
        double[] qVals = {0.1, 0.2, 0.25, 0.4, 0.5};
        int[] labels = {0, 0, 1, 1, 1};
        double[] params = trainLogistic(qVals, labels, 0.5, 200);
        double w = params[0];
        double b = params[1];
        double qTest = 0.3;
        double pHat = predict(qTest, w, b);
        System.out.println("Predicted success probability: " + pHat);
    }
}
      

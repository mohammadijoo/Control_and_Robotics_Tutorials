
public class JointDiscModel {
    private final double[][] Phi;  // 2x2
    private final double[] Gamma;  // 2x1

    public JointDiscModel(double J, double b, double Ts) {
        double[][] A = {
            {0.0,        1.0},
            {0.0, -b / J}
        };
        double[] B = {0.0, 1.0 / J};

        Phi = new double[2][2];
        double[][] I = {
            {1.0, 0.0},
            {0.0, 1.0}
        };

        // Phi = I + Ts*A
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                Phi[i][j] = I[i][j] + Ts * A[i][j];
            }
        }

        Gamma = new double[2];
        for (int i = 0; i < 2; ++i) {
            Gamma[i] = Ts * B[i];
        }
    }

    public void step(double[] x, double u) {
        double x0 = Phi[0][0] * x[0] + Phi[0][1] * x[1] + Gamma[0] * u;
        double x1 = Phi[1][0] * x[0] + Phi[1][1] * x[1] + Gamma[1] * u;
        x[0] = x0;
        x[1] = x1;
    }

    public static void main(String[] args) {
        double J = 0.01, b = 0.1, Ts = 1e-3;
        JointDiscModel model = new JointDiscModel(J, b, Ts);
        double[] x = {0.0, 0.0};
        double u = 1.0;

        for (int k = 0; k < 1000; ++k) {
            model.step(x, u);
            // publish x to ROSJava topics, log, etc.
        }
    }
}

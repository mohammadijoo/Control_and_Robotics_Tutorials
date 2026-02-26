import java.util.Arrays;

public class CableState {
    public double[][] q; // positions [M+1][3]

    public CableState(int M) {
        q = new double[M + 1][3];
    }

    public CableState copy() {
        CableState c = new CableState(q.length - 1);
        for (int i = 0; i < q.length; ++i) {
            c.q[i] = Arrays.copyOf(q[i], 3);
        }
        return c;
    }
}

public class BendingIntegrator {
    private final double step;

    public BendingIntegrator(double step) {
        this.step = step;
    }

    // gradient descent on E_bend
    public void stepOnce(CableState state) {
        int M = state.q.length - 1;
        double[][] grad = new double[M + 1][3];

        for (int k = 1; k <= M - 1; ++k) {
            double[] qkm1 = state.q[k - 1];
            double[] qk   = state.q[k];
            double[] qkp1 = state.q[k + 1];

            double[] d2 = new double[3];
            for (int c = 0; c < 3; ++c) {
                d2[c] = qkp1[c] - 2.0 * qk[c] + qkm1[c];
            }

            // E_k = ||d2||^2, grad wrt q_{k-1}, q_k, q_{k+1}
            for (int c = 0; c < 3; ++c) {
                double g = 2.0 * d2[c];
                grad[k + 1][c] += g;
                grad[k][c]     += -2.0 * g;
                grad[k - 1][c] += g;
            }
        }

        // gradient step
        for (int k = 0; k <= M; ++k) {
            for (int c = 0; c < 3; ++c) {
                state.q[k][c] -= step * grad[k][c];
            }
        }
    }
}
      

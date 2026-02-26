public class DoubleIntegratorTrajOpt {

    private final int N;
    private final double T;
    private final double h;

    public DoubleIntegratorTrajOpt(int N, double T) {
        this.N = N;
        this.T = T;
        this.h = T / N;
    }

    // Layout: w = [x0(2), u0(1), x1(2), u1(1), ..., u_{N-1}(1), xN(2)]
    private int idxX(int k) {
        return 3 * k;
    }

    private int idxU(int k) {
        return 3 * k + 2;
    }

    public double cost(double[] w) {
        double J = 0.0;
        for (int k = 0; k < N; ++k) {
            int iu = idxU(k);
            double ak = w[iu];
            J += 0.5 * h * ak * ak;
        }
        return J;
    }

    public void dynamicsResiduals(double[] w, double[] g) {
        // g has length 2 * N
        for (int k = 0; k < N; ++k) {
            int ix = idxX(k);
            int iu = idxU(k);

            double pk = w[ix];
            double vk = w[ix + 1];
            double ak = w[iu];

            double pkp1 = w[ix + 3];
            double vkp1 = w[ix + 4];

            double pNext = pk + h * vk;
            double vNext = vk + h * ak;

            g[2 * k]     = pkp1 - pNext;
            g[2 * k + 1] = vkp1 - vNext;
        }
    }

    public static void main(String[] args) {
        int N = 40;
        double T = 2.0;
        DoubleIntegratorTrajOpt problem = new DoubleIntegratorTrajOpt(N, T);

        double[] w = new double[3 * N + 2];
        double[] g = new double[2 * N];

        double J = problem.cost(w);
        problem.dynamicsResiduals(w, g);

        System.out.println("Initial cost: " + J);
    }
}
      

public class CollectiveTransport2D {
    private int N;
    private double kp, kv, alpha, beta, m, dt;
    private double[] c = new double[2];
    private double[] v = new double[2];
    private double[] goal = new double[2];
    private double[] f;
    private double[][] L;

    public CollectiveTransport2D(int N, double kp, double kv,
                                 double alpha, double beta,
                                 double mass, double dt) {
        this.N = N;
        this.kp = kp;
        this.kv = kv;
        this.alpha = alpha;
        this.beta = beta;
        this.m = mass;
        this.dt = dt;
        this.goal[0] = 1.0;
        this.goal[1] = 0.0;
        this.f = new double[N];
        for (int i = 0; i < N; i++) {
            f[i] = 1.0 / N;
        }
        this.L = ringLaplacian(N);
    }

    private double[][] ringLaplacian(int N) {
        double[][] L = new double[N][N];
        for (int i = 0; i < N; i++) {
            L[i][i] = 2.0;
            L[i][(i - 1 + N) % N] = -1.0;
            L[i][(i + 1) % N] = -1.0;
        }
        return L;
    }

    public void step() {
        double[] e = {c[0] - goal[0], c[1] - goal[1]};
        double[] Fd = {-kp * e[0] - kv * v[0],
                       -kp * e[1] - kv * v[1]};
        double normFd = Math.sqrt(Fd[0] * Fd[0] + Fd[1] * Fd[1]);
        if (normFd < 1e-6) return;

        double[] d = {Fd[0] / normFd, Fd[1] / normFd};
        double Fd_mag = normFd;

        double[] g = new double[N];
        double base = Fd_mag / N;
        for (int i = 0; i < N; i++) {
            g[i] = f[i] - base;
        }

        double[] gdot = new double[N];
        for (int i = 0; i < N; i++) {
            double sum = 0.0;
            for (int j = 0; j < N; j++) {
                sum += L[i][j] * g[j];
            }
            gdot[i] = -alpha * sum - beta * g[i];
        }

        for (int i = 0; i < N; i++) {
            f[i] += dt * gdot[i];
            if (f[i] < 0.0) f[i] = 0.0;
            if (f[i] > 5.0) f[i] = 5.0;
        }

        double Fsum = 0.0;
        for (int i = 0; i < N; i++) {
            Fsum += f[i];
        }
        double[] Fnet = {d[0] * Fsum, d[1] * Fsum};

        double[] a = {Fnet[0] / m, Fnet[1] / m};
        v[0] += dt * a[0];
        v[1] += dt * a[1];
        c[0] += dt * v[0];
        c[1] += dt * v[1];
    }

    public double[] getPosition() {
        return c;
    }

    public static void main(String[] args) {
        CollectiveTransport2D sim =
            new CollectiveTransport2D(6, 2.0, 0.5, 1.0, 1.0, 10.0, 0.01);
        for (int k = 0; k < 1000; k++) {
            sim.step();
        }
        System.out.println("Final position: (" +
                           sim.getPosition()[0] + ", " +
                           sim.getPosition()[1] + ")");
    }
}
      

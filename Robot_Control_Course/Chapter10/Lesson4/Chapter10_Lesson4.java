
public class MpcController {

    private final int nx;
    private final int nu;
    private final int N;
    private final double[][] H;
    private final double[][] F;  // F^T x
    private final double[][] G;
    private final double[] g;
    private final QpSolver solver;

    private double[] uWarm;
    private double[] lambdaWarm;

    public MpcController(int nx, int nu, int N,
                         double[][] H, double[][] F,
                         double[][] G, double[] g,
                         QpSolver solver) {
        this.nx = nx;
        this.nu = nu;
        this.N  = N;
        this.H  = H;
        this.F  = F;
        this.G  = G;
        this.g  = g;
        this.solver = solver;

        int nU = N * nu;
        int nIneq = G.length;
        this.uWarm = new double[nU];
        this.lambdaWarm = new double[nIneq];
    }

    public double[] computeControl(double[] x) {
        int nU = N * nu;
        int nIneq = G.length;

        // q = F^T x
        double[] q = new double[nU];
        for (int i = 0; i != nU; ++i) {
            double s = 0.0;
            for (int j = 0; j != nx; ++j) {
                s += F[j][i] * x[j];
            }
            q[i] = s;
        }

        // Set up QP with warm-start
        QpProblem prob = new QpProblem(H, q, G, g);
        prob.setPrimalWarmStart(uWarm);
        prob.setDualWarmStart(lambdaWarm);

        QpSolution sol = solver.solve(prob);

        double[] Uopt = sol.getPrimal();
        double[] lam  = sol.getDual();

        // Shift warm-start for next step
        for (int i = 0; i != nU - nu; ++i) {
            uWarm[i] = Uopt[i + nu];
        }
        for (int i = nU - nu; i != nU; ++i) {
            uWarm[i] = uWarm[i - nu];
        }
        System.arraycopy(lam, 0, lambdaWarm, 0, nIneq);

        // Return first control move
        double[] u0 = new double[nu];
        System.arraycopy(Uopt, 0, u0, 0, nu);
        return u0;
    }
}

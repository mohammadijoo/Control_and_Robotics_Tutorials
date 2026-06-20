
import org.ejml.simple.SimpleMatrix;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.QuadraticSolver;
import org.ojalgo.optimisation.QuadraticSolver.Builder;

public class JointMPC {

    public static class MPCQP {
        public SimpleMatrix H;
        public SimpleMatrix g;
        public SimpleMatrix A;
        public SimpleMatrix l;
        public SimpleMatrix u;
    }

    public static MPCQP buildQP(SimpleMatrix x0, double qMin, double qMax,
                                double dqMax, double tauMax,
                                double Ts, int N) {
        MPCQP qp = new MPCQP();
        // Build prediction and constraint matrices here
        // ...
        return qp;
    }

    public static double solveAndGetFirstTorque(MPCQP qp) {
        Builder builder = QuadraticSolver.getBuilder(qp.H.numRows());

        // Set quadratic and linear terms
        builder.quadratic(qp.H.getMatrix());
        builder.linear(qp.g.getMatrix());

        // Inequality constraints l <= A z <= u
        // ojAlgo expects these in a specific canonical form;
        // convert qp.A, qp.l, qp.u accordingly.
        // ...

        QuadraticSolver solver = builder.build();
        Optimisation.Result result = solver.solve();

        if (result.getState().isFailure()) {
            throw new RuntimeException("MPC QP infeasible");
        }

        // Extract first control input
        double[] z = result.toRawCopy1D();
        return z[0];
    }

    public static void main(String[] args) {
        double Ts = 0.01;
        int N = 20;
        double qMin = -1.5, qMax = 1.5, dqMax = 2.0, tauMax = 2.0;

        SimpleMatrix x = new SimpleMatrix(2, 1);
        x.set(0, 0, 0.0);
        x.set(1, 0, 0.0);

        for (int k = 0; k < 50; ++k) {
            MPCQP qp = buildQP(x, qMin, qMax, dqMax, tauMax, Ts, N);
            double tau = solveAndGetFirstTorque(qp);

            // Simulate discrete double integrator
            SimpleMatrix A = new SimpleMatrix(new double[][]{
                    {1.0, Ts},
                    {0.0, 1.0}
            });
            SimpleMatrix B = new SimpleMatrix(new double[][]{
                    {0.0},
                    {Ts / 0.05}
            });

            x = A.mult(x).plus(B.scale(tau));

            System.out.println("step " + k
                    + ", q = " + x.get(0)
                    + ", dq = " + x.get(1)
                    + ", tau = " + tau);
        }
    }
}

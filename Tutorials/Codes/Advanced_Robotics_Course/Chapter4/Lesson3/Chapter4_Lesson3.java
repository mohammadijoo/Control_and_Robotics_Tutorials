import org.ejml.simple.SimpleMatrix;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.QuadraticSolver;
import org.ojalgo.optimisation.QuadraticSolver.Builder;

public class TrajOpt1D {

    public static void main(String[] args) {
        int N = 10;
        double lambdaSmooth = 1.0;
        double trustRadius = 0.2;

        // Initial straight-line trajectory q_k in 1D
        double qStart = 0.0;
        double qGoal = 1.0;
        SimpleMatrix q = new SimpleMatrix(N + 1, 1);
        for (int k = 0; k <= N; ++k) {
            double s = (double) k / N;
            q.set(k, 0, (1.0 - s) * qStart + s * qGoal);
        }

        int nvar = N + 1;
        SimpleMatrix H = new SimpleMatrix(nvar, nvar);
        SimpleMatrix g = new SimpleMatrix(nvar, 1);

        // Build Hessian for smoothness cost sum (q_{k+1} - 2 q_k + q_{k-1})^2
        for (int k = 1; k < N; ++k) {
            int iPrev = k - 1;
            int iCurr = k;
            int iNext = k + 1;

            H.set(iPrev, iPrev, H.get(iPrev, iPrev) + lambdaSmooth);
            H.set(iCurr, iCurr, H.get(iCurr, iCurr) + 4.0 * lambdaSmooth);
            H.set(iNext, iNext, H.get(iNext, iNext) + lambdaSmooth);

            H.set(iPrev, iCurr, H.get(iPrev, iCurr) - 2.0 * lambdaSmooth);
            H.set(iCurr, iPrev, H.get(iCurr, iPrev) - 2.0 * lambdaSmooth);

            H.set(iCurr, iNext, H.get(iCurr, iNext) - 2.0 * lambdaSmooth);
            H.set(iNext, iCurr, H.get(iNext, iCurr) - 2.0 * lambdaSmooth);

            H.set(iPrev, iNext, H.get(iPrev, iNext) + 1.0 * lambdaSmooth);
            H.set(iNext, iPrev, H.get(iNext, iPrev) + 1.0 * lambdaSmooth);
        }

        // Goal term (q_N - qGoal)^2
        H.set(N, N, H.get(N, N) + 1.0);
        g.set(N, 0, -2.0 * qGoal);

        // Trust region: |q_k - q_k_current| <= trustRadius
        // This yields linear inequality constraints: q_k_current - trustRadius <= q_k <= q_k_current + trustRadius

        // Use ojAlgo builder
        Builder builder = QuadraticSolver.getBuilder(nvar);
        for (int i = 0; i < nvar; ++i) {
            for (int j = 0; j < nvar; ++j) {
                builder.quadratic(i, j, H.get(i, j) * 0.5); // ojAlgo uses 0.5 x^T Q x
            }
            builder.linear(i, g.get(i, 0));
            double qi = q.get(i, 0);
            builder.lower(i, qi - trustRadius);
            builder.upper(i, qi + trustRadius);
        }

        // Fix endpoints
        builder.lower(0, qStart);
        builder.upper(0, qStart);
        builder.lower(N, qGoal);
        builder.upper(N, qGoal);

        QuadraticSolver solver = builder.build();
        Optimisation.Result result = solver.solve();
        System.out.println("Solver state: " + result.getState());

        double[] xOpt = result.toRawCopy1D();
        System.out.println("Optimized trajectory:");
        for (int i = 0; i < xOpt.length; ++i) {
            System.out.println("q[" + i + "] = " + xOpt[i]);
        }
    }
}
      

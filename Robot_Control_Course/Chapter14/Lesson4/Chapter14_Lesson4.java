
import org.ojalgo.matrix.Primitive64Matrix;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.QuadraticSolver;
import org.ojalgo.optimisation.QuadraticSolver.Builder;

public class WholeBodyQP {

    public static Optimisation.Result solveWeightedQP(
            Primitive64Matrix H,
            Primitive64Matrix g,
            Primitive64Matrix Aeq,
            Primitive64Matrix beq) {

        Builder builder = QuadraticSolver.getBuilder();
        builder.quadratic(H);
        builder.linear(g);

        if (Aeq != null) {
            builder.equalities(Aeq, beq);
        }

        QuadraticSolver solver = builder.build();
        return solver.solve();
    }
}

// Usage (inside a control loop):
// 1. Fill H and g from your tasks (using the same formulas as in Python).
// 2. Call WholeBodyQP.solveWeightedQP(H, g, Aeq, beq).
// 3. Convert the solution vector into joint commands.

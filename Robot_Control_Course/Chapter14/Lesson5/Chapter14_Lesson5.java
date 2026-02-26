
import org.ejml.simple.SimpleMatrix;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.optimisation.Optimisation.Result;

public class WholeBodyQPJava {

    public static double[] solveQP(SimpleMatrix H, SimpleMatrix g,
                                   double[] qddMin, double[] qddMax) {

        int n = H.numRows();
        ExpressionsBasedModel model = new ExpressionsBasedModel();

        Variable[] qddVars = new Variable[n];
        for (int i = 0; i < n; ++i) {
            qddVars[i] = Variable.make("qdd" + i)
                    .lower(qddMin[i])
                    .upper(qddMax[i]);
            model.addVariable(qddVars[i]);
        }

        // Quadratic objective: 0.5 * qdd^T H qdd + g^T qdd
        var expr = model.addExpression("cost");
        for (int i = 0; i < n; ++i) {
            expr.set(qddVars[i], g.get(i, 0));
            for (int j = 0; j < n; ++j) {
                double hij = 0.5 * H.get(i, j);
                expr.setQuadraticFactor(qddVars[i], qddVars[j], hij);
            }
        }
        expr.weight(1.0);

        Result result = model.minimise();
        double[] qdd = new double[n];
        for (int i = 0; i < n; ++i) {
            qdd[i] = result.get(i).doubleValue();
        }
        return qdd;
    }
}

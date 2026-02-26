
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.optimisation.Expression;

public class JointCBFQP1DOF {

    private final double qMin, qMax, gamma, uMin, uMax;

    public JointCBFQP1DOF(double qMin, double qMax,
                          double gamma,
                          double uMin, double uMax) {
        this.qMin = qMin;
        this.qMax = qMax;
        this.gamma = gamma;
        this.uMin = uMin;
        this.uMax = uMax;
    }

    private double nominalVelocity(double q, double qdot,
                                   double qRef) {
        double kp = 20.0;
        double kd = 5.0;
        double e = qRef - q;
        double edot = -qdot;
        return kp * e + kd * edot;
    }

    public double solveQP(double q, double qdot, double qRef) {
        double uNom = nominalVelocity(q, qdot, qRef);

        // CBF bounds
        double cbfLower = -gamma * (q - qMin);    // u >= cbfLower
        double cbfUpper =  gamma * (qMax - q);    // u <= cbfUpper

        ExpressionsBasedModel model = new ExpressionsBasedModel();

        Variable u = Variable.make("u")
                .lower(uMin).upper(uMax)
                .weight(1.0); // for quadratic objective scaling
        model.addVariable(u);

        // Objective: minimize (u - uNom)^2
        Expression objective = model.addExpression("objective");
        objective.setQuadraticFactor(u, u, 1.0);
        objective.setLinearFactor(u, -2.0 * uNom);
        objective.weight(1.0);

        // CBF constraints: cbfLower <= u <= cbfUpper
        Expression lowerCbf = model.addExpression("cbfLower");
        lowerCbf.setLinearFactor(u, 1.0);
        lowerCbf.lower(cbfLower);

        Expression upperCbf = model.addExpression("cbfUpper");
        upperCbf.setLinearFactor(u, 1.0);
        upperCbf.upper(cbfUpper);

        Optimisation.Result result = model.minimise();
        if (result.getState().isFeasible()) {
            return result.get(u).doubleValue();
        } else {
            // Fallback to saturated nominal
            double uSat = Math.max(uMin, Math.min(uMax, uNom));
            return uSat;
        }
    }
}

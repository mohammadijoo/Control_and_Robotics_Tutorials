import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.DormandPrince54Integrator;

public class JointODEExample {

    // Parameters
    static final double J = 0.01;
    static final double b = 0.1;
    static final double k = 1.0;

    public static class JointODE implements FirstOrderDifferentialEquations {

        @Override
        public int getDimension() {
            return 2; // q, qd
        }

        @Override
        public void computeDerivatives(double t, double[] x, double[] dxdt) {
            double q  = x[0];
            double qd = x[1];
            double u  = 1.0; // constant torque
            double qdd = (u - b * qd - k * q) / J;
            dxdt[0] = qd;
            dxdt[1] = qdd;
        }
    }

    public static void main(String[] args) {
        FirstOrderDifferentialEquations ode = new JointODE();

        double t0 = 0.0;
        double tf = 5.0;
        double[] x0 = new double[] {0.0, 0.0};

        double minStep = 1.0e-3;
        double maxStep = 0.05;
        double absTol = 1.0e-8;
        double relTol = 1.0e-8;

        FirstOrderIntegrator integrator =
            new DormandPrince54Integrator(minStep, maxStep, absTol, relTol);

        integrator.addStepHandler((interpolator, isLast) -> {
            double t = interpolator.getCurrentTime();
            double[] x = interpolator.getInterpolatedState();
            System.out.println(t + " " + x[0] + " " + x[1]);
        });

        integrator.integrate(ode, t0, x0, tf, x0);
    }
}
      

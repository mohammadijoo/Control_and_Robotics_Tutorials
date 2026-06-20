import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.DormandPrince54Integrator;

public class StateInputOutputDemo {

    static double u(double t) {
        return (t >= 0.0) ? 1.0 : 0.0;
    }

    static class Dynamics implements FirstOrderDifferentialEquations {
        @Override
        public int getDimension() { return 1; }

        @Override
        public void computeDerivatives(double t, double[] x, double[] dxdt) {
            dxdt[0] = -x[0] + u(t);
        }
    }

    public static void main(String[] args) {
        double t0 = 0.0, tf = 5.0;
        double[] x = new double[] { 0.2 }; // x(0)

        FirstOrderIntegrator integrator =
            new DormandPrince54Integrator(1.0e-8, 1.0e-2, 1.0e-10, 1.0e-10);

        integrator.integrate(new Dynamics(), t0, x, tf, x);

        double y = x[0]; // y = x
        System.out.println("x(tf) = " + x[0]);
        System.out.println("y(tf) = " + y);
    }
}

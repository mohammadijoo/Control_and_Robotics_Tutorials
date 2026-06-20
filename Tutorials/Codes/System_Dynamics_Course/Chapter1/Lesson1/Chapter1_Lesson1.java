
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;

public class SimpleSystem implements FirstOrderDifferentialEquations {

    private final double a;
    private final double b;

    public SimpleSystem(double a, double b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public int getDimension() {
        return 1; // scalar state
    }

    @Override
    public void computeDerivatives(double t, double[] x, double[] dxdt) {
        double u = 1.0; // unit-step input
        dxdt[0] = a * x[0] + b * u;
    }

    public static void main(String[] args) {
        double a = -1.0;
        double b = 1.0;
        SimpleSystem system = new SimpleSystem(a, b);

        FirstOrderIntegrator integrator =
                new ClassicalRungeKuttaIntegrator(0.01);

        double t0 = 0.0;
        double tf = 5.0;
        double[] x0 = new double[]{0.0};

        integrator.integrate(system, t0, x0, tf, x0);
        System.out.println("x(" + tf + ") = " + x0[0]);
    }
}
      
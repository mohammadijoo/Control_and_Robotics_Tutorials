import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;

public class LinearTimeVaryingODE {

  static class LinSys implements FirstOrderDifferentialEquations {
    @Override
    public int getDimension() { return 2; }

    @Override
    public void computeDerivatives(double t, double[] x, double[] dxdt) {
      double a00 = 0.0, a01 = 1.0;
      double a10 = -2.0 - 0.5*Math.sin(t);
      double a11 = -0.4;

      double b0 = 0.0;
      double b1 = Math.cos(2.0*t);

      dxdt[0] = a00*x[0] + a01*x[1] + b0;
      dxdt[1] = a10*x[0] + a11*x[1] + b1;
    }
  }

  public static void main(String[] args) {
    double t0 = 0.0, T = 8.0;
    double[] x = new double[] { 1.0, 0.0 };

    DormandPrince853Integrator integrator =
        new DormandPrince853Integrator(1e-6, 1e-2, 1e-10, 1e-12);

    integrator.addStepHandler(new StepHandler() {
      @Override
      public void init(double t0, double[] x0, double t) { }

      @Override
      public void handleStep(StepInterpolator interpolator, boolean isLast) {
        double t = interpolator.getCurrentTime();
        if (Math.abs(t - Math.rint(t)) < 1e-3) { // near integer times
          double[] state = interpolator.getInterpolatedState();
          System.out.printf("t=%.3f  x=[%.6f, %.6f]%n", t, state[0], state[1]);
        }
      }
    });

    integrator.integrate(new LinSys(), t0, x, T, x);
    System.out.printf("Final state x(T)=[%.8f, %.8f]%n", x[0], x[1]);
  }
}
      

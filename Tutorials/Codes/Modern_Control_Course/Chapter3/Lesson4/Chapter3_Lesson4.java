import java.util.Arrays;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;

public class FundamentalMatrixLTV {

  // Example A(t) for n=2
  static double[][] A(double t) {
    return new double[][]{
      {0.0, 1.0},
      {-(2.0 + 0.1*t), -3.0}
    };
  }

  // Multiply 2x2 by 2x2
  static double[][] matMul(double[][] M, double[][] N) {
    double[][] R = new double[2][2];
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        R[i][j] = 0.0;
        for (int k = 0; k < 2; k++) R[i][j] += M[i][k] * N[k][j];
      }
    }
    return R;
  }

  // Stack columns of Phi into a length-4 vector: [Phi11, Phi21, Phi12, Phi22]
  static double[] vecPhi(double[][] Phi) {
    return new double[]{Phi[0][0], Phi[1][0], Phi[0][1], Phi[1][1]};
  }

  static double[][] unvecPhi(double[] v) {
    return new double[][]{
      {v[0], v[2]},
      {v[1], v[3]}
    };
  }

  public static void main(String[] args) {
    final double t0 = 0.0;
    final double tf = 2.0;

    // Phi(t0)=I
    double[] y0 = new double[]{1.0, 0.0, 0.0, 1.0};

    FirstOrderDifferentialEquations ode = new FirstOrderDifferentialEquations() {
      public int getDimension() { return 4; }

      public void computeDerivatives(double t, double[] y, double[] yDot) {
        double[][] Phi = unvecPhi(y);
        double[][] dPhi = matMul(A(t), Phi);
        double[] v = vecPhi(dPhi);
        System.arraycopy(v, 0, yDot, 0, 4);
      }
    };

    DormandPrince853Integrator integrator =
        new DormandPrince853Integrator(1.0e-6, 0.1, 1.0e-10, 1.0e-12);

    integrator.addStepHandler(new StepHandler() {
      public void init(double t0, double[] y0, double t) {}

      public void handleStep(StepInterpolator interpolator, boolean isLast) {
        double t = interpolator.getCurrentTime();
        double[] y = interpolator.getInterpolatedState();
        if (isLast) {
          double[][] Phi = unvecPhi(y);
          System.out.println("Phi(tf) = " + Arrays.deepToString(Phi));

          // Example IVP x(tf)=Phi(tf)*x0 (since Phi(t0)=I)
          double[] x0 = new double[]{1.0, 0.0};
          double[] x = new double[]{
            Phi[0][0]*x0[0] + Phi[0][1]*x0[1],
            Phi[1][0]*x0[0] + Phi[1][1]*x0[1]
          };
          System.out.println("x(tf) approx = " + Arrays.toString(x));
        }
      }
    });

    double[] y = y0.clone();
    integrator.integrate(ode, t0, y, tf, y);
  }
}

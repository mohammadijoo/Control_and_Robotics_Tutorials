
public interface DiscreteSystem {
    double[] apply(double[] u);
}

// Static nonlinear system: y[k] = u[k]^2
public class StaticNonlinear implements DiscreteSystem {
    @Override
    public double[] apply(double[] u) {
        double[] y = new double[u.length];
        for (int k = 0; k < u.length; ++k) {
            y[k] = u[k] * u[k];
        }
        return y;
    }
}

// Dynamic linear system: y[k+1] = a*y[k] + b*u[k], y[0] = 0
public class FirstOrderDynamic implements DiscreteSystem {
    private final double a;
    private final double b;

    public FirstOrderDynamic(double a, double b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public double[] apply(double[] u) {
        int N = u.length;
        double[] y = new double[N];
        y[0] = 0.0;
        for (int k = 0; k + 1 < N; ++k) {
            y[k + 1] = a * y[k] + b * u[k];
        }
        return y;
    }
}
      
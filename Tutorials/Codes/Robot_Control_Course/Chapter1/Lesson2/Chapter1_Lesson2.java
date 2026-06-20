
public class Joint1DOFStateSpace {
    private final double J;
    private final double b;
    private final double k;

    public interface TorqueFunction {
        double value(double t);
    }

    public Joint1DOFStateSpace(double J, double b, double k) {
        this.J = J;
        this.b = b;
        this.k = k;
    }

    /**
     * x = [q, q_dot]
     * returns x_dot
     */
    public double[] f(double t, double[] x, TorqueFunction u) {
        double q = x[0];
        double qdot = x[1];
        double tau = u.value(t);

        double qddot = (tau - b * qdot - k * q) / J;
        return new double[]{ qdot, qddot };
    }

    public double[][] getA() {
        return new double[][]{
            { 0.0,     1.0 },
            { -k / J, -b / J }
        };
    }

    public double[][] getB() {
        return new double[][]{
            { 0.0 },
            { 1.0 / J }
        };
    }

    public double[][] getC() {
        return new double[][]{
            { 1.0, 0.0 }
        };
    }

    public double[][] getD() {
        return new double[][]{
            { 0.0 }
        };
    }
}

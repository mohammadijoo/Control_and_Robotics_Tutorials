public class FirstOrderLowPass {
    private final double alpha;
    private double yPrev = 0.0;

    /**
     * Continuous-time prototype: H(s) = 1 / (1 + tau s).
     * Discretized with sampling time Ts using simple first-order recursion.
     */
    public FirstOrderLowPass(double tau, double Ts) {
        this.alpha = Ts / (tau + Ts);
    }

    public double filter(double u) {
        yPrev = yPrev + alpha * (u - yPrev);
        return yPrev;
    }
}

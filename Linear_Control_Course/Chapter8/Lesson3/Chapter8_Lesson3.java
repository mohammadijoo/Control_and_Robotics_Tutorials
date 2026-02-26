public class SteadyStateErrorCalculator {

    public enum InputType {
        STEP,
        RAMP,
        PARABOLIC
    }

    public static double steadyStateError(InputType input,
                                          double Kp, double Kv, double Ka) {
        switch (input) {
            case STEP:
                if (Double.isInfinite(Kp)) {
                    return 0.0;
                }
                return 1.0 / (1.0 + Kp);

            case RAMP:
                if (Kv <= 0.0) {
                    return Double.POSITIVE_INFINITY;
                }
                return 1.0 / Kv;

            case PARABOLIC:
                if (Ka <= 0.0) {
                    return Double.POSITIVE_INFINITY;
                }
                return 1.0 / Ka;

            default:
                return Double.NaN;
        }
    }

    public static void main(String[] args) {
        double Kp = 20.0;  // type-1 controller, finite Kp
        double Kv = 5.0;
        double Ka = 0.0;

        System.out.println("Step e_ss      = " +
                steadyStateError(InputType.STEP, Kp, Kv, Ka));
        System.out.println("Ramp e_ss      = " +
                steadyStateError(InputType.RAMP, Kp, Kv, Ka));
        System.out.println("Parabolic e_ss = " +
                steadyStateError(InputType.PARABOLIC, Kp, Kv, Ka));
    }
}

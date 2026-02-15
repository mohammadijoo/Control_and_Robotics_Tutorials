import org.apache.commons.math3.complex.Complex;

public class LeadCompensator {
    private final double Kc;
    private final double alpha;
    private final double T;

    public LeadCompensator(double Kc, double alpha, double T) {
        this.Kc = Kc;
        this.alpha = alpha;
        this.T = T;
    }

    public Complex eval(Complex s) {
        Complex num = Complex.ONE.add(s.multiply(alpha * T));
        Complex den = Complex.ONE.add(s.multiply(T));
        return num.divide(den).multiply(Kc);
    }

    public static Complex plant(Complex s) {
        // Example plant G(s) = 1 / (s (s + 2))
        return Complex.ONE.divide(s.multiply(s.add(2.0)));
    }

    public static void main(String[] args) {
        LeadCompensator C = new LeadCompensator(1.0, 4.0, 0.1);
        double[] omegaList = {0.5, 1.0, 2.0, 5.0};

        for (double omega : omegaList) {
            Complex s = new Complex(0.0, omega);
            Complex L = C.eval(s).multiply(plant(s));

            double mag = L.abs();
            double phaseRad = L.getArgument();
            double phaseDeg = Math.toDegrees(phaseRad);

            System.out.println("omega = " + omega
                + ", |L(j omega)| = " + mag
                + ", phase(L) = " + phaseDeg + " deg");
        }
    }
}

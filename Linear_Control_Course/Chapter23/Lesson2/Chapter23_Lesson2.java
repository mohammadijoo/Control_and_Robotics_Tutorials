import org.apache.commons.math3.complex.Complex;

public class JointPoleUncertainty {

    public static Complex[] poles(double J, double b, double k) {
        // J s^2 + b s + k = 0
        double disc = b*b - 4.0*J*k;
        Complex sqrtDisc = new Complex(disc, 0.0).sqrt();
        Complex s1 = new Complex(-b, 0.0).add(sqrtDisc).divide(2.0*J);
        Complex s2 = new Complex(-b, 0.0).subtract(sqrtDisc).divide(2.0*J);
        return new Complex[]{s1, s2};
    }

    public static void main(String[] args) {
        double J0 = 0.01;
        double b0 = 0.05;
        double k0 = 2.0;
        double rel_unc = 0.2;

        java.util.Random rng = new java.util.Random(42L);

        for (int i = 0; i < 10; ++i) {
            double J = J0 * (1.0 + rel_unc * (2.0*rng.nextDouble() - 1.0));
            double b = b0 * (1.0 + rel_unc * (2.0*rng.nextDouble() - 1.0));
            double k = k0 * (1.0 + rel_unc * (2.0*rng.nextDouble() - 1.0));
            Complex[] ps = poles(J, b, k);
            System.out.println("Sample " + i
                + " poles: " + ps[0] + ", " + ps[1]);
        }
    }
}

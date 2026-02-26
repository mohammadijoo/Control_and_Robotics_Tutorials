import org.apache.commons.math3.complex.Complex;

public class HFUnmodeledDynamics {

    static Complex P0(Complex s, double J, double b, double Km) {
        // Km / (J s^2 + b s)
        Complex denom = s.multiply(s).multiply(J).add(s.multiply(b));
        return new Complex(Km, 0.0).divide(denom);
    }

    static Complex Gf(Complex s, double wF, double zetaF) {
        // wF^2 / (s^2 + 2 zetaF wF s + wF^2)
        Complex denom = s.multiply(s)
                         .add(s.multiply(2.0 * zetaF * wF))
                         .add(new Complex(wF * wF, 0.0));
        return new Complex(wF * wF, 0.0).divide(denom);
    }

    public static void main(String[] args) {
        double J = 0.01;
        double b = 0.1;
        double Km = 1.0;
        double wF = 200.0;
        double zetaF = 0.05;

        double maxDelta = 0.0;
        int N = 400;
        for (int k = 0; k <= N; ++k) {
            double exp = 0.0 + 4.0 * k / (double) N; // 10^0 .. 10^4
            double w = Math.pow(10.0, exp);
            Complex s = new Complex(0.0, w);
            Complex P0jw = P0(s, J, b, Km);
            Complex Gfjw = Gf(s, wF, zetaF);
            Complex DeltaM = Gfjw.subtract(Complex.ONE); // multiplicative deviation
            double mag = DeltaM.abs();
            if (mag > maxDelta) {
                maxDelta = mag;
            }
        }
        System.out.println("Max |Delta_m(jw)| over grid = " + maxDelta);

        // In robot joint velocity loops, this information can be used to
        // reduce controller gain near frequencies with large multiplicative error.
    }
}

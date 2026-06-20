import org.apache.commons.math3.complex.Complex;
import java.util.function.DoubleFunction;

public class MarginExample {

    static Complex evalTF(double[] num, double[] den, double w) {
        Complex s = new Complex(0.0, w);
        Complex N = Complex.ZERO;
        Complex D = Complex.ZERO;

        for (double v : num) {
            N = N.multiply(s).add(v);
        }
        for (double v : den) {
            D = D.multiply(s).add(v);
        }
        return N.divide(D);
    }

    public static void main(String[] args) {

        // Plant G(s) = Kt / (J s^2 + b s)
        double J = 0.01;
        double b = 0.1;
        double Kt = 0.5;
        double[] Gnum = {Kt};
        double[] Gden = {J, b, 0.0};

        // PI controller C(s) = (Kp s + Ki) / s
        double Kp = 20.0;
        double Ki = 10.0;
        double[] Cnum = {Kp, Ki};
        double[] Cden = {1.0, 0.0};

        DoubleFunction<Complex> L = (double w) -> {
            Complex G = evalTF(Gnum, Gden, w);
            Complex C = evalTF(Cnum, Cden, w);
            return G.multiply(C);
        };

        double wMin = 0.1;
        double wMax = 100.0;
        int N = 2000;

        double bestGcDiff = 1e9;
        double bestGc = 0.0;
        double bestPcDiff = 1e9;
        double bestPc = 0.0;
        double phaseAtGc = 0.0;
        double magAtPc = 0.0;

        for (int k = 0; k < N; ++k) {
            double logw = Math.log10(wMin) + (Math.log10(wMax) - Math.log10(wMin)) * k / (N - 1);
            double w = Math.pow(10.0, logw);

            Complex Lw = L.apply(w);
            double mag = Lw.abs();
            double phase = Lw.getArgument(); // radians

            double gcDiff = Math.abs(mag - 1.0);
            if (gcDiff < bestGcDiff) {
                bestGcDiff = gcDiff;
                bestGc = w;
                phaseAtGc = phase;
            }

            double pcDiff = Math.abs(phase + Math.PI);
            if (pcDiff < bestPcDiff) {
                bestPcDiff = pcDiff;
                bestPc = w;
                magAtPc = mag;
            }
        }

        double pm = (Math.PI + phaseAtGc) * 180.0 / Math.PI;
        double gm = (magAtPc > 0.0) ? (1.0 / magAtPc) : Double.POSITIVE_INFINITY;
        double gmDb = 20.0 * Math.log10(gm);

        double pmRad = pm * Math.PI / 180.0;
        double tauMax = pmRad / bestGc;
        double deltaMult = 2.0 * Math.sin(0.5 * pmRad);

        System.out.println("omega_gc ~ " + bestGc + " rad/s");
        System.out.println("omega_pc ~ " + bestPc + " rad/s");
        System.out.println("Phase margin ~ " + pm + " deg");
        System.out.println("Gain margin ~ " + gm + " (" + gmDb + " dB)");
        System.out.println("Delay margin tau_max ~ " + tauMax + " s");
        System.out.println("delta_max (multiplicative) ~ " + deltaMult);

        // Integration with a robot framework like WPILib:
        // the gains Kp, Ki can be tuned so that the analytically computed margins
        // remain acceptable for the range of robot operating conditions.
    }
}

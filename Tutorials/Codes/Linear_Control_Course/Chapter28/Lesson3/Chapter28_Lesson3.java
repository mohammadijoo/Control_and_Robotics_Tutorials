import java.util.ArrayList;
import java.util.List;

public class BodeExample {

    public static void main(String[] args) {
        double J = 0.01;
        double b = 0.1;
        double Km = 1.0;
        double K = 50.0;

        List<Double> omega = new ArrayList<>();
        for (double w = 0.1; w <= 100.0; w *= 1.1) {
            omega.add(w);
        }

        for (double w : omega) {
            // s = j * w
            double real = 0.0;
            double imag = w;

            // G(s) = Km / (s (J s + b))
            // Complex arithmetic by hand:
            // s = j w, J s + b = b + j J w
            double denom1_real = 0.0;
            double denom1_imag = w;
            double denom2_real = b;
            double denom2_imag = J * w;

            // denom = denom1 * denom2
            double denom_real = denom1_real * denom2_real - denom1_imag * denom2_imag;
            double denom_imag = denom1_real * denom2_imag + denom1_imag * denom2_real;

            double G_real = Km * denom_real / (denom_real * denom_real + denom_imag * denom_imag);
            double G_imag = -Km * denom_imag / (denom_real * denom_real + denom_imag * denom_imag);

            double L_real = K * G_real;
            double L_imag = K * G_imag;

            double mag = Math.hypot(L_real, L_imag);
            double phase = Math.toDegrees(Math.atan2(L_imag, L_real));
            double magDb = 20.0 * Math.log10(mag);

            System.out.println(w + " " + magDb + " " + phase);
        }
    }
}

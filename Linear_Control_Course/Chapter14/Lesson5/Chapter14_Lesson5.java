public class BodeExample {
    public static void main(String[] args) {
        double wn = 10.0;
        double zeta = 0.4;
        double K = 2.0;

        double[] omega = {0.5, 1.0, 5.0, 10.0, 20.0, 50.0};

        System.out.println("omega, mag_dB, phase_deg");

        for (double w : omega) {
            // Complex arithmetic via manual real/imag parts
            double sr = 0.0;
            double si = w; // s = j*w

            // G(s) = wn^2 / (s^2 + 2*zeta*wn s + wn^2)
            double numGr = wn * wn;
            double numGi = 0.0;

            // s^2
            double s2r = -w * w;
            double s2i = 0.0;

            // 2*zeta*wn*s
            double t = 2.0 * zeta * wn;
            double tSr = 0.0;
            double tSi = t * w;

            double denGr = s2r + tSr + wn * wn;
            double denGi = s2i + tSi + 0.0;

            double denomNorm2 = denGr * denGr + denGi * denGi;

            double Gr = (numGr * denGr + numGi * denGi) / denomNorm2;
            double Gi = (numGi * denGr - numGr * denGi) / denomNorm2;

            // C(s) = K
            double Cr = K;
            double Ci = 0.0;

            // L(s) = C(s) * G(s)
            double Lr = Cr * Gr - Ci * Gi;
            double Li = Cr * Gi + Ci * Gr;

            double mag = Math.hypot(Lr, Li);
            double mag_dB = 20.0 * Math.log10(mag);
            double phase_rad = Math.atan2(Li, Lr);
            double phase_deg = phase_rad * 180.0 / Math.PI;

            System.out.printf("%6.2f, %8.2f, %8.2f%n",
                              w, mag_dB, phase_deg);
        }
    }
}

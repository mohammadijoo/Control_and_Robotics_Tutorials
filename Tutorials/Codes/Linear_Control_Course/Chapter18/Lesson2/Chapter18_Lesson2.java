public class DisturbanceRejection {

    public static void main(String[] args) {
        double J = 0.01;
        double b = 0.1;
        double Kp = 5.0;
        double Ki = 10.0;

        // Frequency grid (rad/s)
        int N = 21;
        double[] w = new double[N];
        for (int i = 0; i < N; i++) {
            w[i] = Math.pow(10.0, -1.0 + 0.15 * i); // 0.1 ... about 31.6
        }

        System.out.println("# w, |Gd(jw)|");
        for (int i = 0; i < N; i++) {
            double wi = w[i];

            // s = j*wi
            double sr = 0.0;
            double si = wi;

            // P(s) = 1 / (J s^2 + b s)
            // s^2
            double s2r = sr * sr - si * si;
            double s2i = 2 * sr * si;

            double denomPr = J * s2r + b * sr;
            double denomPi = J * s2i + b * si;

            double denomPabs2 = denomPr * denomPr + denomPi * denomPi;
            double Pr = denomPr / denomPabs2;
            double Pi = -denomPi / denomPabs2;

            // C(s) = Kp + Ki/s
            // 1/s
            double sabs2 = sr * sr + si * si;
            double invsr = sr / sabs2;
            double invsi = -si / sabs2;

            double Cr = Kp + Ki * invsr;
            double Ci = Ki * invsi;

            // L = C * P
            double Lr = Cr * Pr - Ci * Pi;
            double Li = Cr * Pi + Ci * Pr;

            // Gd = P / (1 + L)
            double onePlusLr = 1.0 + Lr;
            double onePlusLi = Li;
            double denomGabs2 = onePlusLr * onePlusLr + onePlusLi * onePlusLi;

            // P / (1 + L)
            double Gdr = (Pr * onePlusLr + Pi * onePlusLi) / denomGabs2;
            double Gdi = (Pi * onePlusLr - Pr * onePlusLi) / denomGabs2;

            double mag = Math.hypot(Gdr, Gdi);
            System.out.println(wi + " " + mag);
        }
    }
}

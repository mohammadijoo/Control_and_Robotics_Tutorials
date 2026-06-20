// Simple complex helper class (minimal subset)
class C {
    public final double re, im;
    public C(double r, double i) { re = r; im = i; }

    public C add(C other) { return new C(re + other.re, im + other.im); }
    public C sub(C other) { return new C(re - other.re, im - other.im); }
    public C mul(C other) {
        return new C(re * other.re - im * other.im,
                     re * other.im + im * other.re);
    }
    public C div(C other) {
        double d = other.re * other.re + other.im * other.im;
        return new C((re * other.re + im * other.im) / d,
                     (im * other.re - re * other.im) / d);
    }
    public double abs() { return Math.hypot(re, im); }
    public double arg() { return Math.atan2(im, re); }
}

public class NicholsClosedLoop {

    // Plant: G(s) = K / (s * (Ts + 1))
    public static C GofJw(double K, double T, double w) {
        C jw = new C(0.0, w);
        C denom = jw.mul(jw.mul(new C(T, 0.0))).add(jw); // Ts^2 + s
        // safer: denom = T*(jw^2) + jw
        // but we'll keep it simple here
        return new C(K, 0.0).div(denom);
    }

    public static void main(String[] args) {
        double K = 5.0;
        double T = 0.1;

        double Mr = 0.0;
        double wMr = 0.0;

        for (int i = 0; i <= 400; ++i) {
            double w = Math.pow(10.0, -1.0 + 3.0 * i / 400.0);
            C L = GofJw(K, T, w);
            C Tjw = L.div(L.add(new C(1.0, 0.0)));

            double M = Tjw.abs();
            double psi = Tjw.arg(); // rad
            double Lmag = L.abs();
            double phi = L.arg();
            double Ldb = 20.0 * Math.log10(Lmag);

            if (M > Mr) {
                Mr = M;
                wMr = w;
            }

            // In a robotics framework, (phi, Ldb, M, psi) can be logged per frequency.
        }

        double Mr_db = 20.0 * Math.log10(Mr);
        System.out.printf("Closed-loop peak Mr = %.3f (%.2f dB) at w = %.3f rad/s%n",
                          Mr, Mr_db, wMr);
    }
}

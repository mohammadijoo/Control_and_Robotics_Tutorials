public class MarginDesignExample {

    // Complex number representation (minimal; for illustration only)
    static class C {
        double re, im;
        C(double r, double i) { re = r; im = i; }
        C add(C o) { return new C(re + o.re, im + o.im); }
        C mul(C o) { return new C(re * o.re - im * o.im, re * o.im + im * o.re); }
        C inv() {
            double d = re * re + im * im;
            return new C(re / d, -im / d);
        }
    }

    static C Gplant(C s) {
        // G(s) = 1 / (s (s + 1))
        C one = new C(1.0, 0.0);
        return (new C(1.0, 0.0)).mul(s.mul(s.add(one)).inv());
    }

    public static void main(String[] args) {
        double omegaGc = 0.7;
        C s = new C(0.0, omegaGc); // j * omega

        C Gjw = Gplant(s);
        double magG = Math.hypot(Gjw.re, Gjw.im);
        double K = 1.0 / magG;

        double phaseRad = Math.atan2(Gjw.im, Gjw.re);
        double pmDeg = 180.0 + phaseRad * 180.0 / Math.PI;

        System.out.println("Designed K = " + K);
        System.out.println("Approx PM (deg) = " + pmDeg);

        // In a robotics framework such as WPILib or custom Java control code,
        // this K would be used in the position servo: u = K * (r - y).
    }
}

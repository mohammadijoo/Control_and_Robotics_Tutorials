public final class StaticErrorAnalysis {

    public static int trailingZeros(double[] coeffs, double tol) {
        int k = 0;
        for (int i = coeffs.length - 1; i >= 0; --i) {
            if (Math.abs(coeffs[i]) < tol) {
                k++;
            } else {
                break;
            }
        }
        return k;
    }

    public static double evalPoly(double[] coeffs, double s) {
        double val = 0.0;
        for (double a : coeffs) {
            val = val * s + a;
        }
        return val;
    }

    public static class Result {
        public int type;
        public double Kp, Kv, Ka;
    }

    public static Result compute(double[] num, double[] den, double sEps) {
        Result r = new Result();
        int zNum = trailingZeros(num, 1e-9);
        int zDen = trailingZeros(den, 1e-9);
        r.type = Math.max(zDen - zNum, 0);

        double L0 = evalPoly(num, 0.0) / evalPoly(den, 0.0);
        double Ls = evalPoly(num, sEps) / evalPoly(den, sEps);

        r.Kp = L0;
        r.Kv = sEps * Ls;
        r.Ka = sEps * sEps * Ls;
        return r;
    }

    public static void main(String[] args) {
        // Example: L(s) = 20 (s + 2) / (s^2 (s + 5))  (Type 2)
        double[] num = {20.0, 40.0};      // 20 s + 40
        double[] den = {1.0, 5.0, 0.0, 0.0}; // s^3 + 5 s^2 + 0 s + 0

        Result r = compute(num, den, 1e-6);
        System.out.println("System type = " + r.type);
        System.out.println("Kp = " + r.Kp + ", Kv = " + r.Kv + ", Ka = " + r.Ka);

        // In a robotics middleware layer, this could be combined with
        // Apache Commons Math for more advanced polynomial and matrix operations.
    }
}

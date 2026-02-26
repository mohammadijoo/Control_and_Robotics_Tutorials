public final class Reliability {

    public static double reliabilityExponential(double lambdaD, double t) {
        if (lambdaD < 0.0 || t < 0.0) {
            throw new IllegalArgumentException("lambdaD and t must be non-negative");
        }
        return Math.exp(-lambdaD * t);
    }

    public static double kOutOfN(double Rc, int n, int k) {
        if (Rc < 0.0 || Rc > 1.0) {
            throw new IllegalArgumentException("Rc must be in [0,1]");
        }
        if (k < 1 || k > n) {
            throw new IllegalArgumentException("Need 1 <= k <= n");
        }
        double Rsys = 0.0;
        for (int i = k; i <= n; ++i) {
            double c = comb(n, i);
            Rsys += c * Math.pow(Rc, i) * Math.pow(1.0 - Rc, n - i);
        }
        return Rsys;
    }

    private static double comb(int n, int r) {
        if (r < 0 || r > n) return 0.0;
        if (r == 0 || r == n) return 1.0;
        double c = 1.0;
        for (int i = 1; i <= r; ++i) {
            c *= (double)(n - r + i) / (double)i;
        }
        return c;
    }

    public static void main(String[] args) {
        double lambdaD = 1e-6;
        double TI = 8760.0;

        double Rc = reliabilityExponential(lambdaD, TI);
        double R1 = kOutOfN(Rc, 1, 1);
        double R2 = kOutOfN(Rc, 2, 1);

        System.out.println("Rc(T_I)  = " + Rc);
        System.out.println("R_1oo1   = " + R1);
        System.out.println("R_1oo2   = " + R2);
    }
}
      

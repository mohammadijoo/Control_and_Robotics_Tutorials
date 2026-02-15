public class TransferFunction {
    public double[] num; // numerator (descending powers of s)
    public double[] den; // denominator

    public TransferFunction(double[] num, double[] den) {
        this.num = num;
        this.den = den;
    }
}

public class TFAlgebra {

    public static double[] convolve(double[] a, double[] b) {
        double[] c = new double[a.length + b.length - 1];
        for (int i = 0; i < a.length; ++i) {
            for (int j = 0; j < b.length; ++j) {
                c[i + j] += a[i] * b[j];
            }
        }
        return c;
    }

    public static double[] addPoly(double[] a, double[] b) {
        int n = Math.max(a.length, b.length);
        double[] c = new double[n];
        // Right-align
        for (int i = 0; i < a.length; ++i) {
            c[n - a.length + i] += a[i];
        }
        for (int i = 0; i < b.length; ++i) {
            c[n - b.length + i] += b[i];
        }
        return c;
    }

    public static TransferFunction series(TransferFunction G1,
                                          TransferFunction G2) {
        double[] num = convolve(G1.num, G2.num);
        double[] den = convolve(G1.den, G2.den);
        return new TransferFunction(num, den);
    }

    public static TransferFunction parallel(TransferFunction G1,
                                            TransferFunction G2) {
        double[] den = convolve(G1.den, G2.den);
        double[] num1 = convolve(G1.num, G2.den);
        double[] num2 = convolve(G2.num, G1.den);
        double[] num = addPoly(num1, num2);
        return new TransferFunction(num, den);
    }

    // Negative feedback: G / (1 + G H)
    public static TransferFunction feedback(TransferFunction G,
                                            TransferFunction H) {
        double[] num = convolve(G.num, H.den);
        double[] den1 = convolve(G.den, H.den);
        double[] den2 = convolve(G.num, H.num);
        double[] den = addPoly(den1, den2);
        return new TransferFunction(num, den);
    }

    public static void main(String[] args) {
        // Example: P-controlled motor-like plant
        double J = 0.01, b = 0.1, K = 1.0, Kc = 20.0;

        TransferFunction Gp = new TransferFunction(
            new double[]{K},
            new double[]{J, b, 0.0}
        );
        TransferFunction Gc = new TransferFunction(
            new double[]{Kc},
            new double[]{1.0}
        );
        TransferFunction Gforward = series(Gc, Gp);
        TransferFunction H = new TransferFunction(
            new double[]{1.0},
            new double[]{1.0}
        );
        TransferFunction Tcl = feedback(Gforward, H);

        System.out.print("Closed-loop numerator: ");
        for (double c : Tcl.num) System.out.print(c + " ");
        System.out.println();
        System.out.print("Closed-loop denominator: ");
        for (double c : Tcl.den) System.out.print(c + " ");
        System.out.println();
    }
}

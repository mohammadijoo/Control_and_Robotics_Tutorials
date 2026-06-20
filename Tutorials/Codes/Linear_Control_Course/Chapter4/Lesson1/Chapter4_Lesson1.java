public class TransferFunction {
    // G(s) = num(s) / den(s)
    private double[] num;
    private double[] den;

    public TransferFunction(double[] num, double[] den) {
        this.num = num;
        this.den = den;
    }

    // Evaluate G(s) for complex s = sigma + j*omega
    public Complex eval(Complex s) {
        Complex N = Complex.ZERO;
        Complex D = Complex.ZERO;

        for (int i = 0; i < num.length; i++) {
            int p = num.length - 1 - i;
            N = N.add(s.pow(p).multiply(num[i]));
        }
        for (int i = 0; i < den.length; i++) {
            int p = den.length - 1 - i;
            D = D.add(s.pow(p).multiply(den[i]));
        }
        return N.divide(D);
    }

    public static void main(String[] args) {
        double K = 2.0;
        double T = 0.5;
        double[] num = {K};
        double[] den = {T, 1.0};

        TransferFunction G = new TransferFunction(num, den);
        Complex s = new Complex(0.0, 1.0); // j*1
        Complex Gjw = G.eval(s);
        System.out.println("G(j*1) = " + Gjw);
    }
}

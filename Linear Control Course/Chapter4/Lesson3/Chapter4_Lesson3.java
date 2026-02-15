public class TransferFunction {
    public double[] num; // numerator coefficients
    public double[] den; // denominator coefficients

    public TransferFunction(double[] num, double[] den) {
        this.num = num;
        this.den = den;
    }

    private static double[] polyMul(double[] a, double[] b) {
        double[] c = new double[a.length + b.length - 1];
        for (int i = 0; i < a.length; ++i)
            for (int j = 0; j < b.length; ++j)
                c[i + j] += a[i] * b[j];
        return c;
    }

    private static double[] polyAdd(double[] a, double[] b) {
        int n = Math.max(a.length, b.length);
        double[] c = new double[n];
        for (int i = 0; i < n; ++i) {
            double av = (i < a.length) ? a[i] : 0.0;
            double bv = (i < b.length) ? b[i] : 0.0;
            c[i] = av + bv;
        }
        return c;
    }

    // Unity feedback: T(s) = G(s)/(1 + G(s))
    public TransferFunction unityFeedback() {
        // 1 is den(s)/den(s)
        double[] numG = this.num;
        double[] denG = this.den;
        double[] numOne = denG.clone();  // num of 1 in common denominator
        double[] denOne = denG.clone();

        double[] numSum = polyAdd(numOne, numG); // numerator of 1 + G
        double[] denSum = denG;                  // denominator of 1 + G

        double[] numT = polyMul(numG, denSum);
        double[] denT = polyMul(denG, numSum);
        return new TransferFunction(numT, denT);
    }
}

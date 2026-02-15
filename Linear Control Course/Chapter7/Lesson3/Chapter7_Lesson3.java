public class RouthHurwitz {

    public static class Result {
        public double[][] table;
        public int nRHP;
        public Result(double[][] t, int n) {
            this.table = t;
            this.nRHP = n;
        }
    }

    public static Result routhArray(double[] coeffs, double eps) {
        int n = coeffs.length - 1;
        int m = (n + 2) / 2;
        double[][] R = new double[n + 1][m];

        // First row: a_n, a_(n-2), ...
        int idx = 0;
        for (int j = 0; j < m && idx < coeffs.length; ++j, idx += 2) {
            R[0][j] = coeffs[idx];
        }

        // Second row: a_(n-1), a_(n-3), ...
        idx = 1;
        for (int j = 0; j < m && idx < coeffs.length; ++j, idx += 2) {
            R[1][j] = coeffs[idx];
        }

        // Build remaining rows
        for (int i = 2; i <= n; ++i) {
            if (Math.abs(R[i - 1][0]) < eps) {
                R[i - 1][0] = eps;
            }
            for (int j = 0; j < m - 1; ++j) {
                double a = R[i - 1][0];
                double b = R[i - 2][0];
                double c = R[i - 2][j + 1];
                double d = R[i - 1][j + 1];
                R[i][j] = (a * c - b * d) / a;
            }
        }

        // Count sign changes in first column
        int signChanges = 0;
        for (int i = 0; i < n; ++i) {
            double x1 = R[i][0];
            double x2 = R[i + 1][0];
            int s1 = (x1 > eps) ? 1 : ((x1 < -eps) ? -1 : 1);
            int s2 = (x2 > eps) ? 1 : ((x2 < -eps) ? -1 : 1);
            if (s1 * s2 < 0) {
                signChanges++;
            }
        }

        return new Result(R, signChanges);
    }

    public static void main(String[] args) {
        // Example polynomial: s^3 + 6 s^2 + 8 s + 10
        double[] coeffs = {1.0, 6.0, 8.0, 10.0};
        Result res = routhArray(coeffs, 1e-9);

        System.out.println("Routh array:");
        for (double[] row : res.table) {
            for (double val : row) {
                System.out.print(val + "\t");
            }
            System.out.println();
        }
        System.out.println("Number of RHP roots: " + res.nRHP);
    }
}

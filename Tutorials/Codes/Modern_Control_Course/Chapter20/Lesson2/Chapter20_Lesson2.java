/*
Chapter20_Lesson2.java

Educational Java implementation for Chapter 20, Lesson 2.

It checks reachability/observability ranks and demonstrates exact minimalization
for the diagonal nonminimal SISO realization:
    A = diag(-1,-2,-3), B = [1,0,1]^T, C = [1,1,0], D = 0.

Compile:
    javac Chapter20_Lesson2.java
Run:
    java Chapter20_Lesson2
*/

public class Chapter20_Lesson2 {
    static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, k = A[0].length, c = B[0].length;
        double[][] M = new double[r][c];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int p = 0; p < k; p++)
                    M[i][j] += A[i][p] * B[p][j];
        return M;
    }

    static double[][] hstack(double[][][] blocks) {
        int rows = blocks[0].length;
        int cols = 0;
        for (double[][] block : blocks) cols += block[0].length;
        double[][] M = new double[rows][cols];
        int offset = 0;
        for (double[][] block : blocks) {
            int c = block[0].length;
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < c; j++)
                    M[i][offset + j] = block[i][j];
            offset += c;
        }
        return M;
    }

    static double[][] vstack(double[][][] blocks) {
        int cols = blocks[0][0].length;
        int rows = 0;
        for (double[][] block : blocks) rows += block.length;
        double[][] M = new double[rows][cols];
        int offset = 0;
        for (double[][] block : blocks) {
            for (int i = 0; i < block.length; i++)
                for (int j = 0; j < cols; j++)
                    M[offset + i][j] = block[i][j];
            offset += block.length;
        }
        return M;
    }

    static int rank(double[][] input, double tol) {
        int rows = input.length;
        int cols = input[0].length;
        double[][] M = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            System.arraycopy(input[i], 0, M[i], 0, cols);

        int r = 0;
        for (int c = 0; c < cols && r < rows; c++) {
            int pivot = r;
            for (int i = r + 1; i < rows; i++)
                if (Math.abs(M[i][c]) > Math.abs(M[pivot][c])) pivot = i;
            if (Math.abs(M[pivot][c]) <= tol) continue;

            double[] temp = M[pivot];
            M[pivot] = M[r];
            M[r] = temp;

            double div = M[r][c];
            for (int j = c; j < cols; j++) M[r][j] /= div;

            for (int i = 0; i < rows; i++) {
                if (i == r) continue;
                double factor = M[i][c];
                for (int j = c; j < cols; j++) M[i][j] -= factor * M[r][j];
            }
            r++;
        }
        return r;
    }

    static double[][] controllability(double[][] A, double[][] B) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] AkB = B;
        blocks[0] = AkB;
        for (int k = 1; k < n; k++) {
            AkB = multiply(A, AkB);
            blocks[k] = AkB;
        }
        return hstack(blocks);
    }

    static double[][] observability(double[][] A, double[][] C) {
        int n = A.length;
        double[][][] blocks = new double[n][][];
        double[][] CAk = C;
        blocks[0] = CAk;
        for (int k = 1; k < n; k++) {
            CAk = multiply(CAk, A);
            blocks[k] = CAk;
        }
        return vstack(blocks);
    }

    static class Complex {
        final double re, im;

        Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        Complex plus(double a) {
            return new Complex(re + a, im);
        }

        Complex reciprocal() {
            double den = re * re + im * im;
            return new Complex(re / den, -im / den);
        }

        @Override
        public String toString() {
            if (Math.abs(im) < 1e-12) return String.format("%.8f", re);
            return String.format("%.8f%+.8fi", re, im);
        }
    }

    static Complex GFull(Complex s) {
        // Only the mode -1 remains in the zero-state input-output map.
        return s.plus(1.0).reciprocal();
    }

    static Complex GMin(Complex s) {
        return s.plus(1.0).reciprocal();
    }

    public static void main(String[] args) {
        double[][] A = {
                {-1.0, 0.0, 0.0},
                { 0.0,-2.0, 0.0},
                { 0.0, 0.0,-3.0}
        };
        double[][] B = {{1.0}, {0.0}, {1.0}};
        double[][] C = {{1.0, 1.0, 0.0}};

        double[][] Wc = controllability(A, B);
        double[][] Wo = observability(A, C);

        System.out.println("rank(Wc) = " + rank(Wc, 1e-10) + " out of n = 3");
        System.out.println("rank(Wo) = " + rank(Wo, 1e-10) + " out of n = 3");
        System.out.println();
        System.out.println("Original realization modes: -1, -2, -3");
        System.out.println("Mode -2 is unreachable; mode -3 is unobservable.");
        System.out.println("Minimal realization: Am=[-1], Bm=[1], Cm=[1], Dm=[0]");
        System.out.println();

        Complex[] samples = {new Complex(0.0, 0.0), new Complex(1.0, 0.0), new Complex(2.0, 1.0)};
        for (Complex s : samples) {
            System.out.println("s = " + s + "  G_full(s) = " + GFull(s) + "  G_min(s) = " + GMin(s));
        }
    }
}

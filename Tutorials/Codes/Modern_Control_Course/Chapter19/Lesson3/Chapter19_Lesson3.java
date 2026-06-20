// Chapter19_Lesson3.java
// Kalman Decomposition: block structure of A, B, C
//
// This pure-Java educational implementation uses rank tests and RREF-based
// bases. For production numerical work, use EJML, Apache Commons Math, or JBLAS.

import java.util.Arrays;

public class Chapter19_Lesson3 {
    static final double TOL = 1e-9;

    static class Result {
        double[][] T, Tinv, Abar, Bbar, Cbar;
        int[] dims;
    }

    static double[][] eye(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] copy(double[][] A) {
        double[][] B = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) B[i] = Arrays.copyOf(A[i], A[i].length);
        return B;
    }

    static double[][] mmul(double[][] A, double[][] B) {
        int m = A.length, p = A[0].length, n = B[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int k = 0; k < p; k++)
                for (int j = 0; j < n; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    static double[][] hstack(double[][]... mats) {
        int rows = mats[0].length;
        int cols = 0;
        for (double[][] M : mats) cols += M[0].length;
        double[][] H = new double[rows][cols];
        int c = 0;
        for (double[][] M : mats) {
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < M[0].length; j++)
                    H[i][c + j] = M[i][j];
            c += M[0].length;
        }
        return H;
    }

    static double[][] vstack(double[][]... mats) {
        int cols = mats[0][0].length;
        int rows = 0;
        for (double[][] M : mats) rows += M.length;
        double[][] V = new double[rows][cols];
        int r = 0;
        for (double[][] M : mats) {
            for (int i = 0; i < M.length; i++)
                for (int j = 0; j < cols; j++)
                    V[r + i][j] = M[i][j];
            r += M.length;
        }
        return V;
    }

    static double[][] subCols(double[][] A, int start, int count) {
        double[][] S = new double[A.length][Math.max(count, 0)];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < count; j++)
                S[i][j] = A[i][start + j];
        return S;
    }

    static double[][] negate(double[][] A) {
        double[][] B = copy(A);
        for (int i = 0; i < B.length; i++)
            for (int j = 0; j < B[0].length; j++)
                B[i][j] = -B[i][j];
        return B;
    }

    static double[][] transpose(double[][] A) {
        double[][] T = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                T[j][i] = A[i][j];
        return T;
    }

    static double[][] rref(double[][] A) {
        double[][] R = copy(A);
        int rows = R.length, cols = R[0].length;
        int lead = 0;
        for (int r = 0; r < rows && lead < cols; r++) {
            int i = r;
            while (i < rows && Math.abs(R[i][lead]) < TOL) i++;
            if (i == rows) {
                lead++;
                r--;
                continue;
            }
            double[] tmp = R[r]; R[r] = R[i]; R[i] = tmp;

            double pivot = R[r][lead];
            for (int j = 0; j < cols; j++) R[r][j] /= pivot;

            for (int ii = 0; ii < rows; ii++) {
                if (ii == r) continue;
                double factor = R[ii][lead];
                for (int j = 0; j < cols; j++) R[ii][j] -= factor * R[r][j];
            }
            lead++;
        }
        return R;
    }

    static int rank(double[][] A) {
        double[][] R = rref(A);
        int rank = 0;
        for (int i = 0; i < R.length; i++) {
            boolean nonzero = false;
            for (int j = 0; j < R[0].length; j++)
                if (Math.abs(R[i][j]) > TOL) nonzero = true;
            if (nonzero) rank++;
        }
        return rank;
    }

    static double[][] independentColumns(double[][] candidates) {
        int m = candidates.length;
        double[][] current = new double[m][0];
        int r = 0;
        for (int j = 0; j < candidates[0].length; j++) {
            double[][] col = subCols(candidates, j, 1);
            double[][] trial = hstack(current, col);
            int rt = rank(trial);
            if (rt > r) {
                current = trial;
                r = rt;
            }
        }
        return current;
    }

    static double[][] appendIndependent(double[][] current, double[][] candidates, int targetDim) {
        double[][] M = current;
        int r = rank(M);
        for (int j = 0; j < candidates[0].length; j++) {
            double[][] col = subCols(candidates, j, 1);
            double[][] trial = hstack(M, col);
            int rt = rank(trial);
            if (rt > r) {
                M = trial;
                r = rt;
                if (targetDim >= 0 && r >= targetDim) break;
            }
        }
        return M;
    }

    static double[][] nullspace(double[][] A) {
        double[][] R = rref(A);
        int rows = R.length, cols = R[0].length;
        boolean[] pivot = new boolean[cols];
        int[] pivotRowForCol = new int[cols];
        Arrays.fill(pivotRowForCol, -1);

        for (int i = 0; i < rows; i++) {
            int pc = -1;
            for (int j = 0; j < cols; j++) {
                if (Math.abs(R[i][j]) > TOL) { pc = j; break; }
            }
            if (pc >= 0) {
                pivot[pc] = true;
                pivotRowForCol[pc] = i;
            }
        }

        int freeCount = 0;
        for (int j = 0; j < cols; j++) if (!pivot[j]) freeCount++;
        double[][] N = new double[cols][freeCount];
        int idx = 0;
        for (int f = 0; f < cols; f++) {
            if (pivot[f]) continue;
            N[f][idx] = 1.0;
            for (int p = 0; p < cols; p++) {
                if (pivot[p]) {
                    int row = pivotRowForCol[p];
                    N[p][idx] = -R[row][f];
                }
            }
            idx++;
        }
        return N;
    }

    static double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] Aug = new double[n][2*n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) Aug[i][j] = A[i][j];
            Aug[i][n+i] = 1.0;
        }
        double[][] R = rref(Aug);
        double[][] Inv = new double[n][n];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Inv[i][j] = R[i][n+j];
        return Inv;
    }

    static double[][] controllabilityMatrix(double[][] A, double[][] B) {
        int n = A.length;
        double[][] Ak = eye(n);
        double[][] R = new double[n][0];
        for (int k = 0; k < n; k++) {
            R = hstack(R, mmul(Ak, B));
            Ak = mmul(A, Ak);
        }
        return R;
    }

    static double[][] observabilityMatrix(double[][] A, double[][] C) {
        int n = A.length;
        double[][] Ak = eye(n);
        double[][] O = new double[0][C[0].length];
        for (int k = 0; k < n; k++) {
            double[][] block = mmul(C, Ak);
            O = (O.length == 0) ? block : vstack(O, block);
            Ak = mmul(A, Ak);
        }
        return O;
    }

    static double[][] intersectionBasis(double[][] U, double[][] V) {
        if (U[0].length == 0 || V[0].length == 0) return new double[U.length][0];
        double[][] K = nullspace(hstack(U, negate(V)));
        if (K[0].length == 0) return new double[U.length][0];
        double[][] alpha = new double[U[0].length][K[0].length];
        for (int i = 0; i < U[0].length; i++)
            for (int j = 0; j < K[0].length; j++)
                alpha[i][j] = K[i][j];
        return independentColumns(mmul(U, alpha));
    }

    static double[][] complementInside(double[][] container, double[][] sub) {
        double[][] completed = appendIndependent(sub, container, container[0].length);
        return subCols(completed, sub[0].length, completed[0].length - sub[0].length);
    }

    static Result kalman(double[][] A, double[][] B, double[][] C) {
        int n = A.length;
        double[][] R = independentColumns(controllabilityMatrix(A, B));
        double[][] N = independentColumns(nullspace(observabilityMatrix(A, C)));
        double[][] V2 = intersectionBasis(R, N);
        double[][] V1 = complementInside(R, V2);
        double[][] V4 = complementInside(N, V2);
        double[][] V124 = hstack(V1, V2, V4);
        double[][] completed = appendIndependent(V124, eye(n), n);
        double[][] V3 = subCols(completed, V124[0].length, completed[0].length - V124[0].length);

        double[][] T = hstack(V1, V2, V3, V4);
        double[][] Tinv = inverse(T);

        Result out = new Result();
        out.T = T;
        out.Tinv = Tinv;
        out.Abar = mmul(mmul(Tinv, A), T);
        out.Bbar = mmul(Tinv, B);
        out.Cbar = mmul(C, T);
        out.dims = new int[]{V1[0].length, V2[0].length, V3[0].length, V4[0].length};
        return out;
    }

    static void printMatrix(String name, double[][] A) {
        System.out.println(name + ":");
        for (double[] row : A) {
            for (double v : row) System.out.printf("%10.5f ", v);
            System.out.println();
        }
    }

    public static void main(String[] args) {
        double[][] A = {
            {-1.0, 0.0, 0.7, 0.0},
            { 0.2,-2.0, 0.3,-0.4},
            { 0.0, 0.0,-3.0, 0.0},
            { 0.0, 0.0, 0.6,-4.0}
        };
        double[][] B = {{1.0},{0.5},{0.0},{0.0}};
        double[][] C = {{2.0,0.0,-1.0,0.0}};

        Result kr = kalman(A, B, C);
        System.out.println("Block dimensions [co, c_unobs, unctrl_obs, unctrl_unobs] = "
                           + Arrays.toString(kr.dims));
        printMatrix("Abar", kr.Abar);
        printMatrix("Bbar", kr.Bbar);
        printMatrix("Cbar", kr.Cbar);
    }
}

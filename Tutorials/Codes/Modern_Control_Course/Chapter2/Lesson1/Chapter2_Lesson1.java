import org.ejml.data.DMatrixRMaj;
import java.util.ArrayList;
import java.util.List;

public class RREF {
    public static class Result {
        public DMatrixRMaj R;
        public List<Integer> pivots;
        public Result(DMatrixRMaj R, List<Integer> pivots) {
            this.R = R; this.pivots = pivots;
        }
    }

    public static Result rref(DMatrixRMaj A, double tol) {
        DMatrixRMaj R = A.copy();
        int m = R.numRows;
        int n = R.numCols;

        List<Integer> pivots = new ArrayList<>();
        int row = 0;

        for (int col = 0; col < n && row < m; col++) {
            // Pivot selection
            int pivot = row;
            double maxAbs = Math.abs(R.get(row, col));
            for (int r = row + 1; r < m; r++) {
                double v = Math.abs(R.get(r, col));
                if (v > maxAbs) { maxAbs = v; pivot = r; }
            }
            if (maxAbs < tol) continue;

            // Swap rows
            if (pivot != row) {
                for (int j = 0; j < n; j++) {
                    double tmp = R.get(row, j);
                    R.set(row, j, R.get(pivot, j));
                    R.set(pivot, j, tmp);
                }
            }

            // Normalize pivot row
            double piv = R.get(row, col);
            for (int j = 0; j < n; j++) R.set(row, j, R.get(row, j) / piv);

            // Eliminate other rows
            for (int r = 0; r < m; r++) {
                if (r == row) continue;
                double factor = R.get(r, col);
                for (int j = 0; j < n; j++) {
                    R.set(r, j, R.get(r, j) - factor * R.get(row, j));
                }
            }

            pivots.add(col);
            row++;
        }

        // Clean small values
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                if (Math.abs(R.get(i, j)) < tol) R.set(i, j, 0.0);

        return new Result(R, pivots);
    }

    public static void main(String[] args) {
        DMatrixRMaj V = new DMatrixRMaj(new double[][]{
            {1,2,0,1},
            {0,1,1,1},
            {1,3,1,2},
            {0,0,1,1}
        });

        Result res = rref(V, 1e-10);
        System.out.println("Pivot columns: " + res.pivots);
        System.out.println("Dimension of span: " + res.pivots.size());
        System.out.println("RREF:");
        res.R.print();
    }
}
      

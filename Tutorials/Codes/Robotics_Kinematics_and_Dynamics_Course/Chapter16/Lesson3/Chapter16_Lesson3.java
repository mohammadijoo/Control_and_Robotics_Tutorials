import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

public class ParallelSingularity {

    public static class SingularityInfo {
        public String type;
        public double detA;
        public double detB;
        public int rankA;
        public int rankB;
        public double kappaA;
        public double kappaB;
    }

    public static SingularityInfo classify(double[][] Adata,
                                           double[][] Bdata,
                                           double tol) {
        SingularityInfo info = new SingularityInfo();

        RealMatrix A = MatrixUtils.createRealMatrix(Adata);
        RealMatrix B = MatrixUtils.createRealMatrix(Bdata);

        info.detA = new org.apache.commons.math3.linear.LUDecomposition(A).getDeterminant();
        info.detB = new org.apache.commons.math3.linear.LUDecomposition(B).getDeterminant();

        SingularValueDecomposition svdA = new SingularValueDecomposition(A);
        SingularValueDecomposition svdB = new SingularValueDecomposition(B);

        double[] sA = svdA.getSingularValues();
        double[] sB = svdB.getSingularValues();

        info.rankA = 0;
        info.rankB = 0;
        for (double v : sA) {
            if (v > tol) info.rankA++;
        }
        for (double v : sB) {
            if (v > tol) info.rankB++;
        }

        info.kappaA = (sA[sA.length - 1] > tol)
                      ? sA[0] / sA[sA.length - 1]
                      : Double.POSITIVE_INFINITY;
        info.kappaB = (sB[sB.length - 1] > tol)
                      ? sB[0] / sB[sB.length - 1]
                      : Double.POSITIVE_INFINITY;

        int m = A.getRowDimension();
        boolean typeI  = info.rankB < m;
        boolean typeII = info.rankA < m;

        if (typeI && typeII) {
            info.type = "Type III (combined)";
        } else if (typeI) {
            info.type = "Type I (serial)";
        } else if (typeII) {
            info.type = "Type II (parallel)";
        } else {
            info.type = "Regular";
        }

        return info;
    }

    public static void main(String[] args) {
        double[][] A = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
        double[][] B = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

        SingularityInfo info = classify(A, B, 1e-6);
        System.out.println("Type: " + info.type);
        System.out.println("det(A) = " + info.detA + ", det(B) = " + info.detB);
        System.out.println("rank(A) = " + info.rankA + ", rank(B) = " + info.rankB);
        System.out.println("kappa(A) = " + info.kappaA + ", kappa(B) = " + info.kappaB);
    }
}
      

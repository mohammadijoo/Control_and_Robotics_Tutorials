import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import java.util.List;

public class CentroidalMomentum {

    public static DMatrixRMaj assembleCentroidalMatrix(
            List<DMatrixRMaj> Ilist,
            List<DMatrixRMaj> Jlist) {

        if (Ilist.size() != Jlist.size()) {
            throw new IllegalArgumentException("Ilist and Jlist must have same length");
        }
        int N = Ilist.size();
        int nq = Jlist.get(0).getNumCols();

        DMatrixRMaj A = new DMatrixRMaj(6, nq);
        A.zero();

        for (int i = 0; i < N; ++i) {
            DMatrixRMaj I = Ilist.get(i);  // 6x6
            DMatrixRMaj J = Jlist.get(i);  // 6xnq
            DMatrixRMaj IJ = new DMatrixRMaj(6, nq);
            CommonOps_DDRM.mult(I, J, IJ);
            CommonOps_DDRM.addEquals(A, IJ);
        }
        return A;
    }

    public static DMatrixRMaj centroidalMomentum(
            List<DMatrixRMaj> Ilist,
            List<DMatrixRMaj> Jlist,
            DMatrixRMaj qdot) {

        DMatrixRMaj A = assembleCentroidalMatrix(Ilist, Jlist);
        DMatrixRMaj hG = new DMatrixRMaj(6, 1);
        CommonOps_DDRM.mult(A, qdot, hG);
        return hG;
    }
}
      

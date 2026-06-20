
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import java.util.List;

public class ExcitationChecker {

    public static DMatrixRMaj gramian(List<DMatrixRMaj> phiList, double dt) {
        if (phiList.isEmpty()) {
            return new DMatrixRMaj(0, 0);
        }
        int m = phiList.get(0).getNumRows();
        DMatrixRMaj G = new DMatrixRMaj(m, m);
        G.zero();

        DMatrixRMaj outer = new DMatrixRMaj(m, m);
        DMatrixRMaj phiT = new DMatrixRMaj(1, m);

        for (DMatrixRMaj phi : phiList) {
            // outer = phi * phi^T
            phiT.reshape(1, m);
            CommonOps_DDRM.transpose(phi, phiT);
            CommonOps_DDRM.mult(phi, phiT, outer);
            CommonOps_DDRM.addEquals(G, outer);
        }

        CommonOps_DDRM.scale(dt, G);
        return G;
    }
}

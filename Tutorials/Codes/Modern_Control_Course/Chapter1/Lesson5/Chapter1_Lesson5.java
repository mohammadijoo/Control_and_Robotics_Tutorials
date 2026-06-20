import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

public class Lesson5StateSpace {
  public static void main(String[] args) {
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {0.0,  1.0},
      {-2.0, -3.0}
    });

    EigenDecomposition_F64<DMatrixRMaj> eig =
      DecompositionFactory_DDRM.eig(2, false);

    if(!eig.decompose(A)) {
      throw new RuntimeException("Eigen decomposition failed");
    }

    System.out.println("eig(A) = ");
    for(int i = 0; i < 2; i++) {
      System.out.println(eig.getEigenvalue(i));
    }
  }
}
      

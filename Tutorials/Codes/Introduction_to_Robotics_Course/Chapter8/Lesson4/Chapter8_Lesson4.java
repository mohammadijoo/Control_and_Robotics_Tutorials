import org.ejml.simple.SimpleMatrix;

public class Chapter8_Lesson4 {
  public static void main(String[] args) {
    // Scalar weighted fusion
    SimpleMatrix z = new SimpleMatrix(new double[][]{
      {1.02},
      {0.97},
      {1.10}
    });
    SimpleMatrix w = new SimpleMatrix(new double[][]{
      {4.0},
      {2.0},
      {1.0}
    });
    double xHat = w.transpose().mult(z).get(0) / w.elementSum();
    System.out.println("Scalar fused estimate: " + xHat);

    // Vector WLS fusion
    SimpleMatrix H1 = SimpleMatrix.identity(2);
    SimpleMatrix z1 = new SimpleMatrix(new double[][]{
      {2.0},
      {1.0}
    });

    SimpleMatrix H2 = new SimpleMatrix(new double[][]{{1.0, 0.0}});
    SimpleMatrix z2 = new SimpleMatrix(new double[][]{{1.8}});

    SimpleMatrix W1 = SimpleMatrix.identity(2).scale(5.0);
    SimpleMatrix W2 = new SimpleMatrix(new double[][]{{2.0}});

    SimpleMatrix A = H1.transpose().mult(W1).mult(H1)
                   .plus(H2.transpose().mult(W2).mult(H2));
    SimpleMatrix b = H1.transpose().mult(W1).mult(z1)
                   .plus(H2.transpose().mult(W2).mult(z2));

    SimpleMatrix xHatVec = A.solve(b);
    System.out.println("Vector fused estimate:");
    xHatVec.print();
  }
}

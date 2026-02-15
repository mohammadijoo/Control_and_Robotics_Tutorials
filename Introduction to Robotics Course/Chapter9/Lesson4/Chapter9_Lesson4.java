import org.ejml.simple.SimpleMatrix;

public class HomoTransformDemo {
  public static void main(String[] args) {
    // Rotation about z by 90 degrees
    double c = 0.0, s = 1.0; // cos(90)=0, sin(90)=1
    SimpleMatrix R = new SimpleMatrix(new double[][]{
      {c, -s, 0},
      {s,  c, 0},
      {0,  0, 1}
    });
    SimpleMatrix p = new SimpleMatrix(3,1,true, new double[]{1,2,0});

    SimpleMatrix T = SimpleMatrix.identity(4);
    T.insertIntoThis(0,0,R);
    T.insertIntoThis(0,3,p);

    SimpleMatrix xBh = new SimpleMatrix(4,1,true, new double[]{1,0,0,1});
    SimpleMatrix xAh = T.mult(xBh);

    System.out.println("x_A = " + xAh.extractMatrix(0,3,0,1));

    // Inverse
    SimpleMatrix RT = R.transpose();
    SimpleMatrix Tinv = SimpleMatrix.identity(4);
    Tinv.insertIntoThis(0,0,RT);
    Tinv.insertIntoThis(0,3, RT.scale(-1).mult(p));

    System.out.println("T*Tinv =\n" + T.mult(Tinv));
  }
}

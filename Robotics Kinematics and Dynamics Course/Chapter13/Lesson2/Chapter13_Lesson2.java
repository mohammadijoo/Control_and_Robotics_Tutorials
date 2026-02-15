import org.ejml.simple.SimpleMatrix;

public class SpatialInertia {
    private double m;
    private SimpleMatrix c;   // 3x1
    private SimpleMatrix Ic;  // 3x3

    public SpatialInertia(double mass, SimpleMatrix com, SimpleMatrix IcCom) {
        this.m = mass;
        this.c = com;
        this.Ic = IcCom;
    }

    public static SimpleMatrix skew(SimpleMatrix v) {
        double vx = v.get(0);
        double vy = v.get(1);
        double vz = v.get(2);
        double[][] data = {
                {0.0,   -vz,   vy},
                {vz,    0.0,  -vx},
                {-vy,   vx,   0.0}
        };
        return new SimpleMatrix(data);
    }

    public SimpleMatrix matrix() {
        SimpleMatrix S = skew(c);
        SimpleMatrix I3 = SimpleMatrix.identity(3);

        SimpleMatrix I11 = Ic.minus(S.mult(S).scale(m));
        SimpleMatrix I12 = S.scale(m);
        SimpleMatrix I21 = S.scale(-m);
        SimpleMatrix I22 = I3.scale(m);

        SimpleMatrix Is = new SimpleMatrix(6, 6);
        Is.insertIntoThis(0, 0, I11);
        Is.insertIntoThis(0, 3, I12);
        Is.insertIntoThis(3, 0, I21);
        Is.insertIntoThis(3, 3, I22);
        return Is;
    }

    public SimpleMatrix momentum(SimpleMatrix v) {
        // v is 6x1
        return matrix().mult(v);
    }

    public double kineticEnergy(SimpleMatrix v) {
        SimpleMatrix h = momentum(v);
        return 0.5 * v.dot(h);
    }

    public static void main(String[] args) {
        double m = 5.0;
        SimpleMatrix c = new SimpleMatrix(new double[][] {
                {0.0}, {0.0}, {0.2}
        });
        SimpleMatrix Ic = SimpleMatrix.diag(0.1, 0.2, 0.15);

        SpatialInertia I = new SpatialInertia(m, c, Ic);
        SimpleMatrix Is = I.matrix();
        SimpleMatrix v = new SimpleMatrix(new double[][] {
                {0.0}, {0.0}, {1.0}, {0.1}, {0.0}, {0.0}
        });

        SimpleMatrix h = I.momentum(v);
        double T = I.kineticEnergy(v);

        System.out.println("Is =\n" + Is);
        System.out.println("h =\n" + h);
        System.out.println("T = " + T);
    }
}
      

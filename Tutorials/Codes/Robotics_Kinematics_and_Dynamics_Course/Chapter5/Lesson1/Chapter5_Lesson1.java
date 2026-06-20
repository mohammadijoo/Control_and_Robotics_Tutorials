import org.ejml.simple.SimpleMatrix;
import java.util.List;
import java.util.ArrayList;

public class PoEForwardKinematics {

    public static SimpleMatrix skew3(SimpleMatrix omega) {
        double wx = omega.get(0);
        double wy = omega.get(1);
        double wz = omega.get(2);
        double[][] data = {
            {0.0,   -wz,   wy},
            {wz,    0.0,  -wx},
            {-wy,   wx,   0.0}
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix matrixExp3(SimpleMatrix omega, double theta) {
        SimpleMatrix wx = skew3(omega);
        SimpleMatrix wx2 = wx.mult(wx);
        return SimpleMatrix.identity(3)
                .plus(wx.scale(Math.sin(theta)))
                .plus(wx2.scale(1.0 - Math.cos(theta)));
    }

    public static SimpleMatrix matrixExp6(SimpleMatrix S, double theta) {
        SimpleMatrix omega = S.rows(0, 3);
        SimpleMatrix v = S.rows(3, 6);

        double normW = omega.normF();
        SimpleMatrix R, p;

        if (normW > 1e-8) {
            SimpleMatrix omegaUnit = omega.divide(normW);
            double thetaScaled = normW * theta;
            R = matrixExp3(omegaUnit, thetaScaled);

            SimpleMatrix wx = skew3(omegaUnit);
            SimpleMatrix wx2 = wx.mult(wx);

            SimpleMatrix G = SimpleMatrix.identity(3).scale(thetaScaled)
                    .plus(wx.scale(1.0 - Math.cos(thetaScaled)))
                    .plus(wx2.scale(thetaScaled - Math.sin(thetaScaled)));

            p = G.mult(v.divide(normW));
        } else {
            R = SimpleMatrix.identity(3);
            p = v.scale(theta);
        }

        SimpleMatrix T = SimpleMatrix.identity(4);
        T.insertIntoThis(0, 0, R);
        T.insertIntoThis(0, 3, p);
        return T;
    }

    public static SimpleMatrix fkineSpace(SimpleMatrix M,
                                          List<SimpleMatrix> Slist,
                                          double[] theta) {
        SimpleMatrix T = SimpleMatrix.identity(4);
        for (int i = 0; i < Slist.size(); ++i) {
            T = T.mult(matrixExp6(Slist.get(i), theta[i]));
        }
        return T.mult(M);
    }

    public static void main(String[] args) {
        double L1 = 1.0;
        double L2 = 1.0;

        SimpleMatrix S1 = new SimpleMatrix(6, 1, true,
                new double[]{0.0, 0.0, 1.0, 0.0,  0.0, 0.0});
        SimpleMatrix S2 = new SimpleMatrix(6, 1, true,
                new double[]{0.0, 0.0, 1.0, 0.0, -L1, 0.0});

        ArrayList<SimpleMatrix> Slist = new ArrayList<>();
        Slist.add(S1);
        Slist.add(S2);

        SimpleMatrix M = SimpleMatrix.identity(4);
        M.set(0, 3, L1 + L2);

        double[] theta = {
            Math.toRadians(30.0),
            Math.toRadians(45.0)
        };

        SimpleMatrix T = fkineSpace(M, Slist, theta);
        T.print();
    }
}
      

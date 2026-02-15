import org.ejml.simple.SimpleMatrix;

public class JacobianSE3 {

    public static SimpleMatrix skew(SimpleMatrix v) {
        double vx = v.get(0), vy = v.get(1), vz = v.get(2);
        double[][] data = {
            {0.0, -vz,  vy},
            {vz,  0.0, -vx},
            {-vy, vx,   0.0}
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix se3Hat(SimpleMatrix S) {
        SimpleMatrix M = SimpleMatrix.identity(4);
        M.zero();
        SimpleMatrix omega = S.rows(0, 3);
        SimpleMatrix v = S.rows(3, 6);
        M.insertIntoThis(0, 0, skew(omega));
        M.insertIntoThis(0, 3, v);
        return M;
    }

    public static SimpleMatrix se3Exp(SimpleMatrix S, double theta) {
        double tol = 1e-9;
        SimpleMatrix omega = S.rows(0, 3);
        SimpleMatrix v = S.rows(3, 6);
        double wnorm = omega.normF();
        SimpleMatrix T = SimpleMatrix.identity(4);
        if (wnorm < tol) {
            // pure translation
            SimpleMatrix p = v.scale(theta);
            T.insertIntoThis(0, 3, p);
            return T;
        }
        SimpleMatrix w = omega.scale(1.0 / wnorm);
        SimpleMatrix w_hat = skew(w);
        double th = wnorm * theta;

        SimpleMatrix I3 = SimpleMatrix.identity(3);
        SimpleMatrix R = I3.plus(w_hat.scale(Math.sin(th)))
                           .plus(w_hat.mult(w_hat).scale(1.0 - Math.cos(th)));

        SimpleMatrix G = I3.scale(th)
                .plus(w_hat.scale(1.0 - Math.cos(th)))
                .plus(w_hat.mult(w_hat).scale(th - Math.sin(th)));

        SimpleMatrix p = G.mult(v.scale(1.0 / wnorm));
        T.insertIntoThis(0, 0, R);
        T.insertIntoThis(0, 3, p);
        return T;
    }

    public static SimpleMatrix adjoint(SimpleMatrix T) {
        SimpleMatrix R = T.extractMatrix(0, 3, 0, 3);
        SimpleMatrix p = T.extractMatrix(0, 3, 3, 4);
        SimpleMatrix Ad = new SimpleMatrix(6, 6);
        Ad.insertIntoThis(0, 0, R);
        Ad.insertIntoThis(3, 3, R);
        Ad.insertIntoThis(3, 0, skew(p).mult(R));
        return Ad;
    }

    public static SimpleMatrix fkPoeSpace(SimpleMatrix[] Slist,
                                          SimpleMatrix M,
                                          double[] q) {
        SimpleMatrix T = SimpleMatrix.identity(4);
        for (int i = 0; i < Slist.length; ++i) {
            T = T.mult(se3Exp(Slist[i], q[i]));
        }
        return T.mult(M);
    }

    public static SimpleMatrix jacobianSpace(SimpleMatrix[] Slist, double[] q) {
        int n = Slist.length;
        SimpleMatrix J = new SimpleMatrix(6, n);
        SimpleMatrix T = SimpleMatrix.identity(4);
        for (int i = 0; i < n; ++i) {
            if (i == 0) {
                J.insertIntoThis(0, 0, Slist[0]);
            } else {
                SimpleMatrix AdT = adjoint(T);
                SimpleMatrix col = AdT.mult(Slist[i]);
                J.insertIntoThis(0, i, col);
            }
            T = T.mult(se3Exp(Slist[i], q[i]));
        }
        return J;
    }

    public static SimpleMatrix jacobianBody(SimpleMatrix[] Slist,
                                            SimpleMatrix M,
                                            double[] q) {
        SimpleMatrix T = fkPoeSpace(Slist, M, q);
        SimpleMatrix Js = jacobianSpace(Slist, q);
        SimpleMatrix AdInv = adjoint(T.invert());
        return AdInv.mult(Js);
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.8;
        SimpleMatrix S1 = new SimpleMatrix(6, 1, true,
                new double[]{0, 0, 1, 0, 0, 0});
        SimpleMatrix S2 = new SimpleMatrix(6, 1, true,
                new double[]{0, 0, 1, 0, -l1, 0});
        SimpleMatrix[] Slist = new SimpleMatrix[]{S1, S2};

        SimpleMatrix M = SimpleMatrix.identity(4);
        M.set(0, 3, l1 + l2);

        double[] q = new double[]{0.5, -0.4};
        SimpleMatrix T = fkPoeSpace(Slist, M, q);
        SimpleMatrix Js = jacobianSpace(Slist, q);
        SimpleMatrix Jb = jacobianBody(Slist, M, q);

        System.out.println("T(q) = \n" + T);
        System.out.println("J_s(q) = \n" + Js);
        System.out.println("J_b(q) = \n" + Jb);
    }
}
      

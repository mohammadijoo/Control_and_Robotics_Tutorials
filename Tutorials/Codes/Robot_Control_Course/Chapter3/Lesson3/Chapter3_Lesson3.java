
import org.ejml.simple.SimpleMatrix;

class TwoLinkParams {
    double l1, l2;
    double m1, m2;
    double I1, I2;
    double g;
}

public class TwoLinkComputedTorque {

    static SimpleMatrix M(SimpleMatrix q, TwoLinkParams p) {
        double q2 = q.get(1);
        double c2 = Math.cos(q2);
        double m11 = p.I1 + p.I2 + p.m1*(p.l1*p.l1)/4.0
                     + p.m2*(p.l1*p.l1 + (p.l2*p.l2)/4.0 + p.l1*p.l2*c2);
        double m12 = p.I2 + p.m2*((p.l2*p.l2)/4.0 + 0.5*p.l1*p.l2*c2);
        double m22 = p.I2 + p.m2*(p.l2*p.l2)/4.0;
        double[][] data = { {m11, m12},
                           {m12, m22} };
        return new SimpleMatrix(data);
    }

    static SimpleMatrix C(SimpleMatrix q, SimpleMatrix qd, TwoLinkParams p) {
        double q2  = q.get(1);
        double q1d = qd.get(0);
        double q2d = qd.get(1);
        double s2  = Math.sin(q2);
        double c11 = -p.m2*p.l1*p.l2*s2*q2d;
        double c12 = -p.m2*p.l1*p.l2*s2*(q1d + q2d);
        double c21 =  p.m2*p.l1*p.l2*s2*q1d;
        double c22 = 0.0;
        double[][] data = { {c11, c12},
                           {c21, c22} };
        return new SimpleMatrix(data);
    }

    static SimpleMatrix g(SimpleMatrix q, TwoLinkParams p) {
        double q1 = q.get(0);
        double q2 = q.get(1);
        double g1 = (p.m1*p.l1/2.0 + p.m2*p.l1)*p.g*Math.cos(q1)
                    + p.m2*p.l2/2.0*p.g*Math.cos(q1 + q2);
        double g2 = p.m2*p.l2/2.0*p.g*Math.cos(q1 + q2);
        return new SimpleMatrix(new double[][]{ {g1},{g2} });
    }

    public static void main(String[] args) {
        TwoLinkParams trueP = new TwoLinkParams();
        trueP.l1 = 1.0; trueP.l2 = 1.0;
        trueP.m1 = 3.0; trueP.m2 = 2.0;
        trueP.I1 = 0.2; trueP.I2 = 0.1;
        trueP.g  = 9.81;

        TwoLinkParams nomP = new TwoLinkParams();
        nomP.l1 = 1.0; nomP.l2 = 1.0;
        nomP.m1 = 2.5; nomP.m2 = 1.5;
        nomP.I1 = 0.2; nomP.I2 = 0.1;
        nomP.g  = 9.81;

        SimpleMatrix Kp = SimpleMatrix.diag(25.0, 16.0);
        SimpleMatrix Kd = SimpleMatrix.diag(10.0, 8.0);

        double dt = 0.001;
        SimpleMatrix q  = new SimpleMatrix(2,1);
        SimpleMatrix qd = new SimpleMatrix(2,1);

        for (int k = 0; k < 10000; ++k) {
            double t = k*dt;
            double qd1 = 0.5*Math.sin(0.5*t);
            double qd2 = 0.3*Math.sin(0.5*t);
            double qd1d = 0.5*0.5*Math.cos(0.5*t);
            double qd2d = 0.3*0.5*Math.cos(0.5*t);
            double qd1dd = -0.5*0.5*0.5*Math.sin(0.5*t);
            double qd2dd = -0.3*0.5*0.5*Math.sin(0.5*t);

            SimpleMatrix qd_d   = new SimpleMatrix(new double[][]{ {qd1},{qd2} });
            SimpleMatrix qd1_d  = new SimpleMatrix(new double[][]{ {qd1d},{qd2d} });
            SimpleMatrix qd2_d  = new SimpleMatrix(new double[][]{ {qd1dd},{qd2dd} });

            SimpleMatrix e  = q.minus(qd_d);
            SimpleMatrix ed = qd.minus(qd1_d);

            SimpleMatrix Mn = M(q, nomP);
            SimpleMatrix Cn = C(q, qd, nomP);
            SimpleMatrix gn = g(q, nomP);

            SimpleMatrix v   = qd2_d.minus(Kd.mult(ed)).minus(Kp.mult(e));
            SimpleMatrix tau = Mn.mult(v).plus(Cn.mult(qd)).plus(gn);

            SimpleMatrix Mr = M(q, trueP);
            SimpleMatrix Cr = C(q, qd, trueP);
            SimpleMatrix gr = g(q, trueP);

            SimpleMatrix qdd = Mr.solve(tau.minus(Cr.mult(qd)).minus(gr));
            q  = q.plus(qd.scale(dt));
            qd = qd.plus(qdd.scale(dt));
        }

        q.print("Final q:");
    }
}

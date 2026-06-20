import org.ejml.simple.SimpleMatrix;

public class FloatingBaseLeg {

    public static class LegParams {
        public double m_b, m1, m2;
        public double I_b, I1, I2;
        public double l1, l2, c1, c2;
        public double g;
    }

    // M(q)
    static SimpleMatrix massMatrix(SimpleMatrix q, LegParams p) {
        // TODO: implement from planar model or call native robotics library
        return SimpleMatrix.identity(5);
    }

    // h(q, dq)
    static SimpleMatrix biasTerm(SimpleMatrix q, SimpleMatrix dq, LegParams p) {
        // TODO: implement gravity and Coriolis from model
        return new SimpleMatrix(5, 1);
    }

    // J_c(q)
    static SimpleMatrix contactJacobian(SimpleMatrix q, LegParams p) {
        // TODO: analytical J_c for planar leg
        return new SimpleMatrix(2, 5);
    }

    // Jdot_c(q, dq) * dq
    static SimpleMatrix contactJdotV(SimpleMatrix q, SimpleMatrix dq, LegParams p) {
        // TODO: implement analytic expression or finite differences
        return new SimpleMatrix(2, 1);
    }

    public static class DynamicsResult {
        public SimpleMatrix ddq;
        public SimpleMatrix lambda;
    }

    public static DynamicsResult constrainedDynamics(SimpleMatrix q,
                                                     SimpleMatrix dq,
                                                     SimpleMatrix tau,
                                                     LegParams p) {
        SimpleMatrix M = massMatrix(q, p);
        SimpleMatrix h = biasTerm(q, dq, p);
        SimpleMatrix Jc = contactJacobian(q, p);
        SimpleMatrix JcDotV = contactJdotV(q, dq, p);

        // Selection matrix S (2 x 5)
        double[][] Sdata = {
                {0, 0, 0, 1, 0},
                {0, 0, 0, 0, 1}
        };
        SimpleMatrix S = new SimpleMatrix(Sdata);

        // Build block matrix A (7 x 7)
        SimpleMatrix A = new SimpleMatrix(7, 7);
        A.insertIntoThis(0, 0, M);
        A.insertIntoThis(0, 5, Jc.transpose().scale(-1.0));
        A.insertIntoThis(5, 0, Jc);

        // Right-hand side
        SimpleMatrix rhs = new SimpleMatrix(7, 1);
        rhs.insertIntoThis(0, 0, S.transpose().mult(tau).minus(h));
        rhs.insertIntoThis(5, 0, JcDotV.negative());

        SimpleMatrix sol = A.solve(rhs);

        DynamicsResult res = new DynamicsResult();
        res.ddq = sol.extractMatrix(0, 5, 0, 1);
        res.lambda = sol.extractMatrix(5, 7, 0, 1);
        return res;
    }

    public static void main(String[] args) {
        LegParams p = new LegParams();
        p.m_b = 10.0; p.m1 = 3.0; p.m2 = 2.0;
        p.I_b = 1.0;  p.I1 = 0.2; p.I2 = 0.1;
        p.l1 = 0.5;   p.l2 = 0.5;
        p.c1 = 0.25;  p.c2 = 0.25;
        p.g  = 9.81;

        SimpleMatrix q  = new SimpleMatrix(new double[][] { {0.0},{0.9},{0.0},{0.4},{-0.8} });
        SimpleMatrix dq = new SimpleMatrix(5,1);
        SimpleMatrix tau = new SimpleMatrix(new double[][] { {0.0},{0.0} });

        DynamicsResult res = constrainedDynamics(q, dq, tau, p);
        System.out.println("ddq = " + res.ddq.transpose());
        System.out.println("lambda = " + res.lambda.transpose());
    }
}
      

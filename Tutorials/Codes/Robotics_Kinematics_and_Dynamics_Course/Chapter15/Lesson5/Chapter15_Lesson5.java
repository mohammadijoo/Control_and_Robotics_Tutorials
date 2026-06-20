import org.ejml.simple.SimpleMatrix;

public class ConstrainedMassOnCircle {

    double m = 1.0;
    double g = 9.81;
    double R = 1.0;
    double alpha = 10.0;
    double beta = 20.0;

    double phi(SimpleMatrix q) {
        double x = q.get(0);
        double y = q.get(1);
        return x * x + y * y - R * R;
    }

    SimpleMatrix J(SimpleMatrix q) {
        double x = q.get(0);
        double y = q.get(1);
        // 1x2 row
        return new SimpleMatrix(1, 2, true, new double[]{2.0 * x, 2.0 * y});
    }

    SimpleMatrix Jdot(SimpleMatrix v) {
        double vx = v.get(0);
        double vy = v.get(1);
        return new SimpleMatrix(1, 2, true, new double[]{2.0 * vx, 2.0 * vy});
    }

    void step(SimpleMatrix q, SimpleMatrix v, double h) {
        SimpleMatrix M = SimpleMatrix.identity(2).scale(m);
        SimpleMatrix gVec = new SimpleMatrix(2, 1, true, new double[]{0.0, m * g});

        SimpleMatrix Jq = J(q);      // 1x2
        SimpleMatrix Jd = Jdot(v);   // 1x2

        double phiVal = phi(q);
        double Jv = Jq.mult(v).get(0);
        double Jdv = Jd.mult(v).get(0);

        // Build 3x3 block matrix A
        SimpleMatrix A = new SimpleMatrix(3, 3);
        // Top-left 2x2
        A.insertIntoThis(0, 0, M);
        // Top-right 2x1 = J^T
        A.set(0, 2, Jq.get(0));
        A.set(1, 2, Jq.get(1));
        // Bottom-left 1x2
        A.set(2, 0, Jq.get(0));
        A.set(2, 1, Jq.get(1));

        // rhs vector
        SimpleMatrix rhs = new SimpleMatrix(3, 1);
        rhs.set(0, 0, -gVec.get(0));
        rhs.set(1, 0, -gVec.get(1));
        rhs.set(2, 0, -Jdv - 2.0 * alpha * Jv - beta * beta * phiVal);

        SimpleMatrix sol = A.solve(rhs);
        SimpleMatrix vdot = sol.rows(0, 2);
        // double lambda = sol.get(2);

        v = v.plus(vdot.scale(h));
        q = q.plus(v.scale(h));

        // Update references (caller holds q, v)
        q.set(0, 0, q.get(0));
        q.set(1, 0, q.get(1));
        v.set(0, 0, v.get(0));
        v.set(1, 0, v.get(1));
    }

    public static void main(String[] args) {
        ConstrainedMassOnCircle sys = new ConstrainedMassOnCircle();
        double h = 1e-3;
        int nSteps = 10000;

        SimpleMatrix q = new SimpleMatrix(2, 1, true, new double[]{sys.R, 0.0});
        SimpleMatrix v = new SimpleMatrix(2, 1, true, new double[]{0.0, 0.5});

        for (int k = 0; k < nSteps; ++k) {
            sys.step(q, v, h);
        }
        System.out.println("Final phi(q) = " + sys.phi(q));
    }
}
      

import org.ejml.simple.SimpleMatrix;

public class BaumgarteMass2D {

    static class Params {
        double m = 1.0;
        double g = 9.81;
        double L = 1.0;
        double alpha = 5.0;
        double beta = 10.0;
    }

    static class State {
        double x, y, vx, vy;
    }

    public static double[] deriv(double t, double[] y, Params p) {
        double x  = y[0];
        double yPos = y[1];
        double vx = y[2];
        double vy = y[3];

        SimpleMatrix M = SimpleMatrix.identity(2).scale(p.m);
        SimpleMatrix h = new SimpleMatrix(2, 1, true, new double[]{0.0, p.m * p.g});

        double phi = x * x + yPos * yPos - p.L * p.L;

        SimpleMatrix J = new SimpleMatrix(1, 2, true, new double[]{2.0 * x, 2.0 * yPos});
        SimpleMatrix Jdot = new SimpleMatrix(1, 2, true, new double[]{2.0 * vx, 2.0 * vy});
        SimpleMatrix qdot = new SimpleMatrix(2, 1, true, new double[]{vx, vy});

        // Build K (3x3)
        SimpleMatrix K = new SimpleMatrix(3, 3);
        K.insertIntoThis(0, 0, M);
        K.insertIntoThis(0, 2, J.transpose().scale(-1.0));
        K.insertIntoThis(2, 0, J);

        // RHS
        SimpleMatrix rhs = new SimpleMatrix(3, 1);
        rhs.insertIntoThis(0, 0, h.scale(-1.0));
        double rhsCon = -(Jdot.mult(qdot).get(0, 0)
                          + 2.0 * p.alpha * J.mult(qdot).get(0, 0)
                          + p.beta * p.beta * phi);
        rhs.set(2, 0, rhsCon);

        // Solve
        SimpleMatrix sol = K.solve(rhs);
        double ax = sol.get(0, 0);
        double ay = sol.get(1, 0);

        return new double[]{vx, vy, ax, ay};
    }

    // Use any ODE integrator (e.g., fixed-step RK4) calling deriv(t, y, p).
}
      

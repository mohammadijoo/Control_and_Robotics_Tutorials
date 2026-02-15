
public class TwoDofLab {

    static final double m1 = 1.0, m2 = 1.0;
    static final double l1 = 1.0, l2 = 1.0;
    static final double lc1 = 0.5, lc2 = 0.5;
    static final double I1 = 0.1, I2 = 0.1;
    static final double g = 9.81;

    static double[][] M(double[] q) {
        double q1 = q[0], q2 = q[1];
        double c2 = Math.cos(q2);
        double m11 = I1 + I2 + m1*lc1*lc1
                     + m2*(l1*l1 + lc2*lc2 + 2*l1*lc2*c2);
        double m12 = I2 + m2*(lc2*lc2 + l1*lc2*c2);
        double m22 = I2 + m2*lc2*lc2;
        return new double[][] { {m11, m12},
                               {m12, m22} };
    }

    static double[][] C(double[] q, double[] qdot) {
        double q2 = q[1];
        double q1dot = qdot[0], q2dot = qdot[1];
        double s2 = Math.sin(q2);
        double h = -m2*l1*lc2*s2;
        return new double[][] {
            {h*q2dot, h*(q1dot + q2dot)},
            {-h*q1dot, 0.0}
        };
    }

    static double[] gvec(double[] q) {
        double q1 = q[0], q2 = q[1];
        double g1 = (m1*lc1 + m2*l1)*g*Math.cos(q1)
                    + m2*lc2*g*Math.cos(q1 + q2);
        double g2 = m2*lc2*g*Math.cos(q1 + q2);
        return new double[] {g1, g2};
    }

    static double[] qd(double t) {
        return new double[] {0.5*Math.sin(0.5*t),
                             0.5*Math.cos(0.5*t)};
    }

    static double[] qdDot(double t) {
        return new double[] {0.25*Math.cos(0.5*t),
                             -0.25*Math.sin(0.5*t)};
    }

    static double[] qdDdot(double t) {
        return new double[] {-0.125*Math.sin(0.5*t),
                             -0.125*Math.cos(0.5*t)};
    }

    static double[][] Kp = {
        {100.0, 0.0},
        {0.0, 80.0}
    };
    static double[][] Kd = {
        {20.0, 0.0},
        {0.0, 16.0}
    };

    static double[] matVec(double[][] A, double[] x) {
        return new double[] {
            A[0][0]*x[0] + A[0][1]*x[1],
            A[1][0]*x[0] + A[1][1]*x[1]
        };
    }

    static double[] vecAdd(double[] a, double[] b) {
        return new double[] {a[0] + b[0], a[1] + b[1]};
    }

    static double[] vecSub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1]};
    }

    static double dot(double[] a, double[] b) {
        return a[0]*b[0] + a[1]*b[1];
    }

    // 2x2 linear solve using explicit formula
    static double[] solve2x2(double[][] A, double[] b) {
        double a11 = A[0][0], a12 = A[0][1];
        double a21 = A[1][0], a22 = A[1][1];
        double det = a11*a22 - a12*a21;
        double inv11 = a22 / det;
        double inv12 = -a12 / det;
        double inv21 = -a21 / det;
        double inv22 = a11 / det;
        return new double[] {
            inv11*b[0] + inv12*b[1],
            inv21*b[0] + inv22*b[1]
        };
    }

    static double[] controllerPD(double[] q, double[] qdot, double t) {
        double[] e = vecSub(qd(t), q);
        double[] edot = vecSub(qdDot(t), qdot);
        double[] term1 = matVec(Kp, e);
        double[] term2 = matVec(Kd, edot);
        return vecAdd(term1, term2);
    }

    static double[] controllerCT(double[] q, double[] qdot, double t) {
        double[] e = vecSub(qd(t), q);
        double[] edot = vecSub(qdDot(t), qdot);
        double[] v = vecAdd(qdDdot(t),
                            vecAdd(matVec(Kd, edot), matVec(Kp, e)));
        double[][] M = M(q);
        double[][] Cmat = C(q, qdot);
        double[] g_vec = gvec(q);
        double[] Cv = matVec(Cmat, qdot);
        double[] tmp = vecAdd(matVec(M, v), vecAdd(Cv, g_vec));
        return tmp;
    }

    static class Metrics {
        double Je, Jtau, emax;
    }

    static Metrics simulate(boolean useCT, double T, double dt) {
        int steps = (int)(T / dt);
        double[] q = {0.0, 0.0};
        double[] qdot = {0.0, 0.0};

        double Je = 0.0, Jtau = 0.0, emax = 0.0;

        for (int k = 0; k < steps; ++k) {
            double t = k*dt;
            double[] e = vecSub(qd(t), q);
            double[] edot = vecSub(qdDot(t), qdot);
            double[] tau = useCT ? controllerCT(q, qdot, t)
                                 : controllerPD(q, qdot, t);

            double[][] M = M(q);
            double[][] Cmat = C(q, qdot);
            double[] g_vec = gvec(q);
            double[] rhs = vecSub(tau, vecAdd(matVec(Cmat, qdot), g_vec));
            double[] qddot = solve2x2(M, rhs);

            qdot[0] += dt*qddot[0];
            qdot[1] += dt*qddot[1];
            q[0] += dt*qdot[0];
            q[1] += dt*qdot[1];

            Je += dot(e, e)*dt;
            Jtau += dot(tau, tau)*dt;
            double en = Math.sqrt(dot(e, e));
            if (en > emax) emax = en;
        }

        Metrics m = new Metrics();
        m.Je = Je; m.Jtau = Jtau; m.emax = emax;
        return m;
    }

    public static void main(String[] args) {
        double T = 10.0, dt = 0.001;
        Metrics mPD = simulate(false, T, dt);
        Metrics mCT = simulate(true, T, dt);
        System.out.println("PD: J_e=" + mPD.Je + " J_tau=" + mPD.Jtau + " e_max=" + mPD.emax);
        System.out.println("CT: J_e=" + mCT.Je + " J_tau=" + mCT.Jtau + " e_max=" + mCT.emax);
    }
}

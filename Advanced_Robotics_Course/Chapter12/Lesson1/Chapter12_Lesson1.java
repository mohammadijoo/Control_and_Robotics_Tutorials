public interface ContinuousMDP {
    int stateDim();
    int actionDim();

    double[] step(double[] x, double[] u, StepResult result);
}

class StepResult {
    public double reward;
    public boolean done;
}

public class SimpleRobotMDP implements ContinuousMDP {
    private final int n;   // dof
    private final int m;   // actuation dimension
    private final double dt;
    private final double[] qLimit;
    private final double[] uLimit;

    public SimpleRobotMDP(int nDof, int mAct, double dt) {
        this.n = nDof;
        this.m = mAct;
        this.dt = dt;
        this.qLimit = new double[n];
        this.uLimit = new double[m];
        for (int i = 0; i < n; ++i) qLimit[i] = Math.PI;
        for (int j = 0; j < m; ++j) uLimit[j] = 10.0;
    }

    @Override
    public int stateDim() { return 2 * n; }

    @Override
    public int actionDim() { return m; }

    private double[] dynamics(double[] x, double[] u) {
        double[] q = new double[n];
        double[] dq = new double[n];
        for (int i = 0; i < n; ++i) {
            q[i] = x[i];
            dq[i] = x[i + n];
        }

        double[] uClip = new double[m];
        for (int j = 0; j < m; ++j) {
            uClip[j] = Math.max(-uLimit[j], Math.min(uLimit[j], u[j]));
        }

        double[] ddq = new double[n];
        for (int i = 0; i < n; ++i) {
            ddq[i] = uClip[Math.min(i, m - 1)];
        }

        double[] qNext = new double[n];
        double[] dqNext = new double[n];
        for (int i = 0; i < n; ++i) {
            qNext[i] = q[i] + dt * dq[i];
            dqNext[i] = dq[i] + dt * ddq[i];
            qNext[i] = Math.max(-qLimit[i], Math.min(qLimit[i], qNext[i]));
        }

        double[] xNext = new double[2 * n];
        for (int i = 0; i < n; ++i) {
            xNext[i] = qNext[i];
            xNext[i + n] = dqNext[i];
        }
        return xNext;
    }

    private double reward(double[] x, double[] u) {
        // Quadratic penalty toward zero state and small actions
        double cost = 0.0;
        for (int i = 0; i < x.length; ++i) {
            cost += x[i] * x[i];
        }
        for (int j = 0; j < u.length; ++j) {
            cost += 0.01 * u[j] * u[j];
        }
        return -cost;
    }

    @Override
    public double[] step(double[] x, double[] u, StepResult result) {
        double[] xNext = dynamics(x, u);
        result.reward = reward(x, u);
        double norm = 0.0;
        for (int i = 0; i < xNext.length; ++i) {
            norm += xNext[i] * xNext[i];
        }
        result.done = (norm > 1e6);
        return xNext;
    }
}
      


public class RobotEKFJava {
    private final int n;
    private final double dt;
    private Mat x;  // state [q; dq], column vector
    private Mat P;  // covariance
    private Mat Q;
    private Mat R;
    private final int dimZ;

    public RobotEKFJava(int nDof, double dt) {
        this.n = nDof;
        this.dt = dt;
        int dimX = 2 * n;

        this.x = Mat.zeros(dimX, 1);
        this.P = Mat.eye(dimX).scale(1e-2);

        this.Q = Mat.zeros(dimX, dimX);
        this.Q.setBlock(0, 0, n, n, Mat.eye(n).scale(1e-6));
        this.Q.setBlock(n, n, n, n, Mat.eye(n).scale(1e-4));

        this.dimZ = n + 3;
        this.R = Mat.zeros(dimZ, dimZ);
        this.R.setBlock(0, 0, n, n, Mat.eye(n).scale(1e-5));
        this.R.setBlock(n, n, 3, 3, Mat.eye(3).scale(1e-4));
    }

    // Process model
    private Mat f(Mat xk, Mat u) {
        int dimX = xk.rows();
        Mat q = xk.subMat(0, n, 0, 1);
        Mat dq = xk.subMat(n, 2 * n, 0, 1);

        Mat qddot = robotForwardDynamics(q, dq, u); // user-defined

        Mat qNext = q.add(dq.scale(dt));
        Mat dqNext = dq.add(qddot.scale(dt));

        Mat xNext = Mat.zeros(dimX, 1);
        xNext.setBlock(0, 0, n, 1, qNext);
        xNext.setBlock(n, 0, n, 1, dqNext);
        return xNext;
    }

    private Mat FJacobian(Mat xk, Mat u) {
        int dimX = xk.rows();
        Mat F = Mat.zeros(dimX, dimX);
        Mat fx = f(xk, u);
        double eps = 1e-6;

        for (int i = 0; i < dimX; ++i) {
            Mat dx = Mat.zeros(dimX, 1);
            dx.set(i, 0, eps);
            Mat fPlus = f(xk.add(dx), u);
            Mat col = fPlus.sub(fx).scale(1.0 / eps);
            F.setCol(i, col);
        }
        return F;
    }

    // Measurement model
    private Mat h(Mat xk) {
        Mat z = Mat.zeros(dimZ, 1);
        Mat q = xk.subMat(0, n, 0, 1);
        z.setBlock(0, 0, n, 1, q);

        Mat ee = forwardKinematics(q); // size 3x1
        z.setBlock(n, 0, 3, 1, ee);
        return z;
    }

    private Mat HJacobian(Mat xk) {
        int dimX = xk.rows();
        Mat H = Mat.zeros(dimZ, dimX);
        Mat h0 = h(xk);
        double eps = 1e-6;

        for (int i = 0; i < dimX; ++i) {
            Mat dx = Mat.zeros(dimX, 1);
            dx.set(i, 0, eps);
            Mat hPlus = h(xk.add(dx));
            Mat col = hPlus.sub(h0).scale(1.0 / eps);
            H.setCol(i, col);
        }
        return H;
    }

    public void predict(Mat u) {
        Mat xPrev = x.copy();
        Mat PPrev = P.copy();

        x = f(xPrev, u);
        Mat F = FJacobian(xPrev, u);
        P = F.mul(PPrev).mul(F.transpose()).add(Q);
    }

    public void update(Mat zMeas) {
        Mat xPred = x.copy();
        Mat PPred = P.copy();

        Mat zPred = h(xPred);
        Mat H = HJacobian(xPred);
        Mat y = zMeas.sub(zPred);

        Mat S = H.mul(PPred).mul(H.transpose()).add(R);
        Mat K = PPred.mul(H.transpose()).mul(S.inverse());

        x = xPred.add(K.mul(y));
        Mat I = Mat.eye(PPred.rows());
        P = (I.sub(K.mul(H)))
                .mul(PPred)
                .mul((I.sub(K.mul(H))).transpose())
                .add(K.mul(R).mul(K.transpose()));
    }

    // Robot-specific stubs:
    private Mat robotForwardDynamics(Mat q, Mat dq, Mat tau) {
        // Implement using your dynamics
        return tau; // qddot = tau (dummy)
    }

    private Mat forwardKinematics(Mat q) {
        Mat ee = Mat.zeros(3, 1);
        double sum = 0.0;
        for (int i = 0; i < q.rows(); ++i)
            sum += q.get(i, 0);
        ee.set(0, 0, sum);
        return ee;
    }

    public Mat getState() { return x; }
}

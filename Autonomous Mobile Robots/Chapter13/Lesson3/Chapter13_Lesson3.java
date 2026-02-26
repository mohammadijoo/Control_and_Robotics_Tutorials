// Chapter13_Lesson3.java
// Visual–Inertial Fusion Pipelines (AMR Focus)
//
// Minimal planar EKF fusion (IMU propagation + visual pose updates).
//
// Dependency (recommended): EJML
//   https://ejml.org/
// Compile example:
//   javac -cp ejml-all-0.43.jar Chapter13_Lesson3.java
// Run:
//   java -cp .:ejml-all-0.43.jar Chapter13_Lesson3

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class Chapter13_Lesson3 {

    static double wrapAngle(double th) {
        th = (th + Math.PI) % (2.0 * Math.PI);
        if (th < 0) th += 2.0 * Math.PI;
        return th - Math.PI;
    }

    static DMatrixRMaj rot2(double th) {
        double c = Math.cos(th), s = Math.sin(th);
        DMatrixRMaj R = new DMatrixRMaj(2, 2);
        R.set(0, 0, c);  R.set(0, 1, -s);
        R.set(1, 0, s);  R.set(1, 1,  c);
        return R;
    }

    static class TruthSample {
        double t;
        double px, py, vx, vy, th;
        double axm, aym, wzm;
    }

    static class VOMeas {
        double t;
        double px, py, th;
    }

    static void simulate(double T, double dt, double vo_dt,
                         ArrayList<TruthSample> samples,
                         ArrayList<VOMeas> vo) {
        Random rng = new Random(7);

        int N = (int)(T / dt) + 1;
        double[] px = new double[N];
        double[] py = new double[N];
        double[] vx = new double[N];
        double[] vy = new double[N];
        double[] th = new double[N];

        for (int k = 1; k < N; k++) {
            double tk = (k - 1) * dt;
            double speed = 1.0 + 0.2 * Math.sin(0.3 * tk);
            double yawRate = 0.25 * Math.sin(0.2 * tk);
            th[k] = wrapAngle(th[k - 1] + yawRate * dt);
            vx[k] = speed * Math.cos(th[k]);
            vy[k] = speed * Math.sin(th[k]);
            px[k] = px[k - 1] + vx[k - 1] * dt;
            py[k] = py[k - 1] + vy[k - 1] * dt;
        }

        double[] axw = new double[N];
        double[] ayw = new double[N];
        for (int k = 1; k < N - 1; k++) {
            axw[k] = (vx[k + 1] - vx[k - 1]) / (2.0 * dt);
            ayw[k] = (vy[k + 1] - vy[k - 1]) / (2.0 * dt);
        }
        axw[0] = axw[1]; ayw[0] = ayw[1];
        axw[N - 1] = axw[N - 2]; ayw[N - 1] = ayw[N - 2];

        double bax = 0.08, bay = -0.05;
        double bg = 0.01;
        double sigmaA = 0.12;
        double sigmaG = 0.015;

        double gx = 0.0, gy = -9.81;

        for (int k = 0; k < N; k++) {
            double tk = k * dt;
            DMatrixRMaj R = rot2(th[k]);

            double awx = axw[k], awy = ayw[k];
            double axg = awx - gx;
            double ayg = awy - gy;

            // a_b = R^T (a_w - g)
            double a_bx = R.get(0,0) * axg + R.get(1,0) * ayg;
            double a_by = R.get(0,1) * axg + R.get(1,1) * ayg;

            double axm = a_bx + bax + sigmaA * rng.nextGaussian();
            double aym = a_by + bay + sigmaA * rng.nextGaussian();
            double yawRate = 0.25 * Math.sin(0.2 * tk);
            double wzm = yawRate + bg + sigmaG * rng.nextGaussian();

            TruthSample s = new TruthSample();
            s.t = tk; s.px = px[k]; s.py = py[k]; s.vx = vx[k]; s.vy = vy[k]; s.th = th[k];
            s.axm = axm; s.aym = aym; s.wzm = wzm;
            samples.add(s);
        }

        int step = (int)(vo_dt / dt);
        double sigmaP = 0.05;
        double sigmaTh = 0.02;
        for (int k = 0; k < N; k += step) {
            VOMeas m = new VOMeas();
            m.t = samples.get(k).t;
            m.px = samples.get(k).px + sigmaP * rng.nextGaussian();
            m.py = samples.get(k).py + sigmaP * rng.nextGaussian();
            m.th = wrapAngle(samples.get(k).th + sigmaTh * rng.nextGaussian());
            vo.add(m);
        }
    }

    static void imuPropagate(DMatrixRMaj x, DMatrixRMaj P,
                             double axm, double aym, double wzm,
                             DMatrixRMaj Qc, double dt) {
        double px = x.get(0,0), py = x.get(1,0), vx = x.get(2,0), vy = x.get(3,0), th = x.get(4,0);
        double bax = x.get(5,0), bay = x.get(6,0), bg = x.get(7,0);

        double gx = 0.0, gy = -9.81;

        double a_bx = axm - bax;
        double a_by = aym - bay;
        double w = wzm - bg;

        double thNew = wrapAngle(th + w * dt);

        double c = Math.cos(th), s = Math.sin(th);
        double awx = c * a_bx - s * a_by + gx;
        double awy = s * a_bx + c * a_by + gy;

        double vxNew = vx + awx * dt;
        double vyNew = vy + awy * dt;
        double pxNew = px + vx * dt + 0.5 * awx * dt * dt;
        double pyNew = py + vy * dt + 0.5 * awy * dt * dt;

        x.set(0,0, pxNew);
        x.set(1,0, pyNew);
        x.set(2,0, vxNew);
        x.set(3,0, vyNew);
        x.set(4,0, thNew);

        DMatrixRMaj F = CommonOps_DDRM.identity(8);
        F.set(0,2, dt);
        F.set(1,3, dt);

        double dawx_dth = (-s) * a_bx - (c) * a_by;
        double dawy_dth = ( c) * a_bx - (s) * a_by;

        F.set(2,4, dawx_dth * dt);
        F.set(3,4, dawy_dth * dt);

        F.set(2,5, -c * dt);
        F.set(2,6,  s * dt);
        F.set(3,5, -s * dt);
        F.set(3,6, -c * dt);

        F.set(0,4, dawx_dth * 0.5 * dt * dt);
        F.set(1,4, dawy_dth * 0.5 * dt * dt);

        F.set(0,5, -c * 0.5 * dt * dt);
        F.set(0,6,  s * 0.5 * dt * dt);
        F.set(1,5, -s * 0.5 * dt * dt);
        F.set(1,6, -c * 0.5 * dt * dt);

        F.set(4,7, -dt);

        DMatrixRMaj FP = new DMatrixRMaj(8,8);
        DMatrixRMaj FPFt = new DMatrixRMaj(8,8);
        CommonOps_DDRM.mult(F, P, FP);
        CommonOps_DDRM.multTransB(FP, F, FPFt);

        DMatrixRMaj Qd = new DMatrixRMaj(8,8);
        CommonOps_DDRM.scale(dt, Qc, Qd);

        CommonOps_DDRM.addEquals(FPFt, Qd);
        P.set(FPFt);
    }

    static void ekfUpdatePose(DMatrixRMaj x, DMatrixRMaj P,
                             double zpx, double zpy, double zth,
                             DMatrixRMaj Rz) {
        DMatrixRMaj H = new DMatrixRMaj(3,8);
        H.set(0,0, 1.0);
        H.set(1,1, 1.0);
        H.set(2,4, 1.0);

        DMatrixRMaj zhat = new DMatrixRMaj(3,1);
        zhat.set(0,0, x.get(0,0));
        zhat.set(1,0, x.get(1,0));
        zhat.set(2,0, x.get(4,0));

        DMatrixRMaj z = new DMatrixRMaj(3,1);
        z.set(0,0, zpx);
        z.set(1,0, zpy);
        z.set(2,0, zth);

        DMatrixRMaj y = new DMatrixRMaj(3,1);
        CommonOps_DDRM.subtract(z, zhat, y);
        y.set(2,0, wrapAngle(y.get(2,0)));

        DMatrixRMaj HP = new DMatrixRMaj(3,8);
        DMatrixRMaj HPHt = new DMatrixRMaj(3,3);
        CommonOps_DDRM.mult(H, P, HP);
        CommonOps_DDRM.multTransB(HP, H, HPHt);
        CommonOps_DDRM.addEquals(HPHt, Rz);

        DMatrixRMaj S_inv = new DMatrixRMaj(3,3);
        CommonOps_DDRM.invert(HPHt, S_inv);

        DMatrixRMaj PHt = new DMatrixRMaj(8,3);
        DMatrixRMaj K = new DMatrixRMaj(8,3);
        CommonOps_DDRM.multTransB(P, H, PHt);
        CommonOps_DDRM.mult(PHt, S_inv, K);

        DMatrixRMaj Ky = new DMatrixRMaj(8,1);
        CommonOps_DDRM.mult(K, y, Ky);
        CommonOps_DDRM.addEquals(x, Ky);
        x.set(4,0, wrapAngle(x.get(4,0)));

        DMatrixRMaj I = CommonOps_DDRM.identity(8);
        DMatrixRMaj KH = new DMatrixRMaj(8,8);
        CommonOps_DDRM.mult(K, H, KH);
        DMatrixRMaj I_KH = new DMatrixRMaj(8,8);
        CommonOps_DDRM.subtract(I, KH, I_KH);

        DMatrixRMaj tmp = new DMatrixRMaj(8,8);
        DMatrixRMaj Pnew = new DMatrixRMaj(8,8);
        CommonOps_DDRM.mult(I_KH, P, tmp);
        CommonOps_DDRM.multTransB(tmp, I_KH, Pnew);

        DMatrixRMaj KRKt = new DMatrixRMaj(8,8);
        DMatrixRMaj KR = new DMatrixRMaj(8,3);
        CommonOps_DDRM.mult(K, Rz, KR);
        CommonOps_DDRM.multTransB(KR, K, KRKt);

        CommonOps_DDRM.addEquals(Pnew, KRKt);
        P.set(Pnew);
    }

    public static void main(String[] args) {
        double dt = 0.01;
        double T = 20.0;
        double voDt = 0.1;

        ArrayList<TruthSample> samples = new ArrayList<>();
        ArrayList<VOMeas> vo = new ArrayList<>();
        simulate(T, dt, voDt, samples, vo);

        DMatrixRMaj x = new DMatrixRMaj(8,1);
        x.set(0,0, vo.get(0).px);
        x.set(1,0, vo.get(0).py);
        x.set(4,0, vo.get(0).th);

        DMatrixRMaj P = new DMatrixRMaj(8,8);
        double[] diag = {0.25, 0.25, 0.04, 0.04, 0.09, 0.04, 0.04, 0.01};
        for (int i=0;i<8;i++) P.set(i,i, diag[i]);

        DMatrixRMaj Qc = new DMatrixRMaj(8,8);
        Qc.set(2,2, 0.4*0.4);
        Qc.set(3,3, 0.4*0.4);
        Qc.set(4,4, 0.15*0.15);
        Qc.set(5,5, 0.01*0.01);
        Qc.set(6,6, 0.01*0.01);
        Qc.set(7,7, 0.002*0.002);

        DMatrixRMaj Rz = new DMatrixRMaj(3,3);
        Rz.set(0,0, 0.05*0.05);
        Rz.set(1,1, 0.05*0.05);
        Rz.set(2,2, 0.02*0.02);

        int voPtr = 0;
        for (int k=0; k<samples.size(); k++) {
            TruthSample s = samples.get(k);
            imuPropagate(x, P, s.axm, s.aym, s.wzm, Qc, dt);

            if (voPtr < vo.size() && Math.abs(s.t - vo.get(voPtr).t) < 0.5*dt) {
                VOMeas m = vo.get(voPtr);
                ekfUpdatePose(x, P, m.px, m.py, m.th, Rz);
                voPtr++;
            }
        }

        TruthSample gt = samples.get(samples.size()-1);
        double ex = x.get(0,0) - gt.px;
        double ey = x.get(1,0) - gt.py;
        double eth = wrapAngle(x.get(4,0) - gt.th);

        System.out.printf("Final error [m, m, rad] = [%.3f, %.3f, %.3f]%n", ex, ey, eth);
    }
}

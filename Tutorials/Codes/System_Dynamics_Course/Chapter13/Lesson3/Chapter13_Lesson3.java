/*
Chapter13_Lesson3.java
System Dynamics (Control Engineering) — Chapter 13, Lesson 3
Modal Coordinates and Decoupling of MDOF Systems (Undamped Case)

This program demonstrates modal decoupling for a 3-DOF mass-spring system.

Key idea (symmetric generalized EVP):
    K phi = lambda M phi, with M symmetric positive definite and K symmetric.

A robust numerical route in Java (with EJML) is:
1) Cholesky factorization: M = L L^T
2) Form symmetric A = L^{-1} K L^{-T}
3) Solve standard symmetric eigenproblem: A y = lambda y
4) Recover generalized eigenvectors: phi = L^{-T} y
5) Mass-normalize so that Phi^T M Phi = I

Integration: RK4 on decoupled modal ODEs.

Dependency:
  EJML (e.g., org.ejml:ejml-all)

Run conceptually (example with gradle/maven):
  javac -cp ejml-all.jar Chapter13_Lesson3.java
  java  -cp .:ejml-all.jar Chapter13_Lesson3
*/
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SpecializedOps_DDRM;
import org.ejml.dense.row.decomposition.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

import java.util.Arrays;

public class Chapter13_Lesson3 {

    static DMatrixRMaj diag(double... d) {
        DMatrixRMaj A = new DMatrixRMaj(d.length, d.length);
        for (int i = 0; i < d.length; i++) A.set(i, i, d[i]);
        return A;
    }

    static void build3DOF(DMatrixRMaj M, DMatrixRMaj K) {
        // wall --k1-- m1 --k2-- m2 --k3-- m3 --k4-- wall
        double m1 = 1.0, m2 = 1.2, m3 = 0.9;
        double k1 = 1200.0, k2 = 900.0, k3 = 700.0, k4 = 1100.0;

        // M
        M.reshape(3,3);
        CommonOps_DDRM.fill(M, 0);
        M.set(0,0,m1); M.set(1,1,m2); M.set(2,2,m3);

        // K
        K.reshape(3,3);
        CommonOps_DDRM.fill(K, 0);
        K.set(0,0,k1+k2); K.set(0,1,-k2);
        K.set(1,0,-k2);   K.set(1,1,k2+k3); K.set(1,2,-k3);
        K.set(2,1,-k3);   K.set(2,2,k3+k4);
    }

    static DMatrixRMaj inverseLowerTriangular(DMatrixRMaj L) {
        // compute inv(L) by solving L X = I
        int n = L.numRows;
        DMatrixRMaj invL = CommonOps_DDRM.identity(n);
        CommonOps_DDRM.solve(L, invL, invL);
        return invL;
    }

    static class ModalResult {
        double[] omega;
        DMatrixRMaj Phi; // columns are mass-normalized modes
    }

    static ModalResult modalDecomposition(DMatrixRMaj M, DMatrixRMaj K) {
        int n = M.numRows;

        // Cholesky: M = L L^T
        CholeskyDecomposition_F64<DMatrixRMaj> chol = DecompositionFactory_DDRM.chol(n, true);
        if (!chol.decompose(M.copy())) {
            throw new RuntimeException("Cholesky failed: M must be SPD.");
        }
        DMatrixRMaj L = chol.getT(null); // lower-triangular

        DMatrixRMaj invL = inverseLowerTriangular(L);

        // A = invL * K * invL^T  (symmetric)
        DMatrixRMaj tmp = new DMatrixRMaj(n,n);
        DMatrixRMaj A = new DMatrixRMaj(n,n);
        CommonOps_DDRM.mult(invL, K, tmp);
        CommonOps_DDRM.multTransB(tmp, invL, A);

        // Symmetrize A to reduce numerical asymmetry
        DMatrixRMaj AT = new DMatrixRMaj(n,n);
        CommonOps_DDRM.transpose(A, AT);
        CommonOps_DDRM.add(0.5, A, 0.5, AT, A);

        // Eigen decomposition of A
        EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, true);
        if (!eig.decompose(A)) {
            throw new RuntimeException("Eigen decomposition failed.");
        }

        double[] lambda = new double[n];
        DMatrixRMaj Y = new DMatrixRMaj(n,n); // eigenvectors y_i as columns
        for (int i = 0; i < n; i++) {
            lambda[i] = eig.getEigenvalue(i).getReal();
            DMatrixRMaj yi = eig.getEigenVector(i);
            for (int r = 0; r < n; r++) Y.set(r, i, yi.get(r,0));
        }

        // Sort by lambda ascending (simple selection sort)
        for (int i = 0; i < n-1; i++) {
            int kmin = i;
            for (int j = i+1; j < n; j++) if (lambda[j] < lambda[kmin]) kmin = j;
            if (kmin != i) {
                double t = lambda[i]; lambda[i] = lambda[kmin]; lambda[kmin] = t;
                // swap columns i and kmin of Y
                for (int r = 0; r < n; r++) {
                    double tmpv = Y.get(r,i);
                    Y.set(r,i, Y.get(r,kmin));
                    Y.set(r,kmin, tmpv);
                }
            }
        }

        // Phi = inv(L^T) * Y  = (invL^T) * Y
        DMatrixRMaj invLT = new DMatrixRMaj(n,n);
        CommonOps_DDRM.transpose(invL, invLT);
        DMatrixRMaj Phi = new DMatrixRMaj(n,n);
        CommonOps_DDRM.mult(invLT, Y, Phi);

        // Mass-normalize columns so that Phi^T M Phi = I
        DMatrixRMaj MPhi = new DMatrixRMaj(n,n);
        CommonOps_DDRM.mult(M, Phi, MPhi);
        for (int i = 0; i < n; i++) {
            // mi = phi_i^T M phi_i
            double mi = 0;
            for (int r = 0; r < n; r++) mi += Phi.get(r,i) * MPhi.get(r,i);
            double s = 1.0/Math.sqrt(mi);
            for (int r = 0; r < n; r++) Phi.set(r,i, Phi.get(r,i)*s);
        }

        double[] omega = new double[n];
        for (int i = 0; i < n; i++) omega[i] = Math.sqrt(Math.max(lambda[i], 0.0));

        ModalResult res = new ModalResult();
        res.omega = omega;
        res.Phi = Phi;
        return res;
    }

    static double[] force(double t, double F0, double Omega) {
        return new double[] {F0*Math.sin(Omega*t), 0.0, 0.0};
    }

    static void rk4Step(double[] q, double[] qd, double[] omega, double[] p, double dt) {
        int n = q.length;

        java.util.function.BiFunction<double[], double[], double[]> acc = (qq, pp) -> {
            double[] qdd = new double[n];
            for (int i = 0; i < n; i++) qdd[i] = -omega[i]*omega[i]*qq[i] + pp[i];
            return qdd;
        };

        // k1
        double[] k1_q = Arrays.copyOf(qd, n);
        double[] k1_qd = acc.apply(q, p);

        // k2
        double[] q2 = new double[n];
        double[] qd2 = new double[n];
        for (int i = 0; i < n; i++) { q2[i] = q[i] + 0.5*dt*k1_q[i]; qd2[i] = qd[i] + 0.5*dt*k1_qd[i]; }
        double[] k2_q = Arrays.copyOf(qd2, n);
        double[] k2_qd = acc.apply(q2, p);

        // k3
        double[] q3 = new double[n];
        double[] qd3 = new double[n];
        for (int i = 0; i < n; i++) { q3[i] = q[i] + 0.5*dt*k2_q[i]; qd3[i] = qd[i] + 0.5*dt*k2_qd[i]; }
        double[] k3_q = Arrays.copyOf(qd3, n);
        double[] k3_qd = acc.apply(q3, p);

        // k4
        double[] q4 = new double[n];
        double[] qd4 = new double[n];
        for (int i = 0; i < n; i++) { q4[i] = q[i] + dt*k3_q[i]; qd4[i] = qd[i] + dt*k3_qd[i]; }
        double[] k4_q = Arrays.copyOf(qd4, n);
        double[] k4_qd = acc.apply(q4, p);

        for (int i = 0; i < n; i++) {
            q[i]  += (dt/6.0)*(k1_q[i]  + 2.0*k2_q[i]  + 2.0*k3_q[i]  + k4_q[i]);
            qd[i] += (dt/6.0)*(k1_qd[i] + 2.0*k2_qd[i] + 2.0*k3_qd[i] + k4_qd[i]);
        }
    }

    static double[] modalForce(DMatrixRMaj Phi, double[] f) {
        // p = Phi^T f  (Phi is n x n, f is n)
        int n = Phi.numRows;
        double[] p = new double[n];
        for (int i = 0; i < n; i++) {
            double s = 0;
            for (int r = 0; r < n; r++) s += Phi.get(r,i) * f[r];
            p[i] = s;
        }
        return p;
    }

    static double[] reconstructX(DMatrixRMaj Phi, double[] q) {
        int n = Phi.numRows;
        double[] x = new double[n];
        for (int r = 0; r < n; r++) {
            double s = 0;
            for (int i = 0; i < n; i++) s += Phi.get(r,i) * q[i];
            x[r] = s;
        }
        return x;
    }

    public static void main(String[] args) {
        DMatrixRMaj M = new DMatrixRMaj(3,3);
        DMatrixRMaj K = new DMatrixRMaj(3,3);
        build3DOF(M, K);

        ModalResult mr = modalDecomposition(M, K);
        System.out.println("Natural frequencies (rad/s): " + Arrays.toString(mr.omega));

        double F0 = 10.0;
        double Omega = 0.9 * mr.omega[0];

        double[] q = new double[] {0,0,0};
        double[] qd = new double[] {0,0,0};

        double t0 = 0.0, tf = 8.0, dt = 1e-3;
        int N = (int)Math.round((tf - t0)/dt);

        for (int step = 0; step <= N; step++) {
            double t = t0 + step*dt;

            double[] f = force(t, F0, Omega);
            double[] p = modalForce(mr.Phi, f);

            if (step % 1000 == 0) {
                double[] x = reconstructX(mr.Phi, q);
                System.out.printf("t=%.3f  x1=%.6f  q1=%.6f%n", t, x[0], q[0]);
            }

            rk4Step(q, qd, mr.omega, p, dt);
        }
    }
}

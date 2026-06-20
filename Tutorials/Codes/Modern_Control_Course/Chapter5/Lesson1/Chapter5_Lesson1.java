import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.function.DoubleFunction;

public class NthOrderOdeToStateSpace {

    static class StateSpace {
        DMatrixRMaj A, B, C, D;
        StateSpace(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj D){
            this.A = A; this.B = B; this.C = C; this.D = D;
        }
    }

    static StateSpace nthOrderOdeToSS(double[] a, double b) {
        int n = a.length;
        DMatrixRMaj A = new DMatrixRMaj(n, n);
        for (int i = 0; i < n - 1; i++) A.set(i, i + 1, 1.0);
        for (int j = 0; j < n; j++) A.set(n - 1, j, -a[j]);

        DMatrixRMaj B = new DMatrixRMaj(n, 1);
        B.set(n - 1, 0, b);

        DMatrixRMaj C = new DMatrixRMaj(1, n);
        C.set(0, 0, 1.0);

        DMatrixRMaj D = new DMatrixRMaj(1, 1); // zero
        return new StateSpace(A, B, C, D);
    }

    static DMatrixRMaj f(StateSpace ss, DoubleFunction<Double> u, double t, DMatrixRMaj x) {
        // dx = A x + B u(t)
        DMatrixRMaj dx = new DMatrixRMaj(x.numRows, 1);
        CommonOps_DDRM.mult(ss.A, x, dx);
        double ut = u.apply(t);
        for (int i = 0; i < x.numRows; i++) {
            dx.add(i, 0, ss.B.get(i, 0) * ut);
        }
        return dx;
    }

    static DMatrixRMaj rk4Step(StateSpace ss, DoubleFunction<Double> u, double t, DMatrixRMaj x, double h) {
        DMatrixRMaj k1 = f(ss, u, t, x);

        DMatrixRMaj x2 = x.copy(); CommonOps_DDRM.addEquals(x2, 0.5*h, k1);
        DMatrixRMaj k2 = f(ss, u, t + 0.5*h, x2);

        DMatrixRMaj x3 = x.copy(); CommonOps_DDRM.addEquals(x3, 0.5*h, k2);
        DMatrixRMaj k3 = f(ss, u, t + 0.5*h, x3);

        DMatrixRMaj x4 = x.copy(); CommonOps_DDRM.addEquals(x4, h, k3);
        DMatrixRMaj k4 = f(ss, u, t + h, x4);

        DMatrixRMaj out = x.copy();
        for (int i = 0; i < out.numRows; i++) {
            double incr = (h/6.0) * (k1.get(i,0) + 2.0*k2.get(i,0) + 2.0*k3.get(i,0) + k4.get(i,0));
            out.add(i, 0, incr);
        }
        return out;
    }

    public static void main(String[] args) {
        // Example: y''' + 3 y'' + 3 y' + 1 y = 2 u
        double[] a = new double[]{1.0, 3.0, 3.0};
        double b = 2.0;
        StateSpace ss = nthOrderOdeToSS(a, b);

        DoubleFunction<Double> u = (t) -> (t >= 0.0) ? 1.0 : 0.0;

        double t0 = 0.0, tf = 10.0, h = 1e-3;
        int N = (int)((tf - t0)/h);

        DMatrixRMaj x = new DMatrixRMaj(3, 1); // [y, y', y''] initially zeros
        double t = t0;

        for (int k = 0; k < N; k++) {
            x = rk4Step(ss, u, t, x, h);
            t += h;
        }

        DMatrixRMaj y = new DMatrixRMaj(1,1);
        CommonOps_DDRM.mult(ss.C, x, y);
        System.out.println("Final y(tf) = " + y.get(0,0));
    }
}

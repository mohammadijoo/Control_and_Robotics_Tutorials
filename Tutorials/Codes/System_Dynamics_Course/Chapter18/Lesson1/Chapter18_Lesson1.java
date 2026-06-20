// Chapter18_Lesson1.java
// Work, Energy, and Power in Mechanical, Electrical, Fluid, and Thermal Systems

public class Chapter18_Lesson1 {

    static double trapz(double[] y, double[] t) {
        double s = 0.0;
        for (int k = 0; k < y.length - 1; k++) {
            s += 0.5 * (y[k] + y[k + 1]) * (t[k + 1] - t[k]);
        }
        return s;
    }

    static double Fm(double tt) {
        return 2.5 * Math.sin(1.2 * tt) + 1.2 * Math.cos(0.4 * tt);
    }

    static double Vin(double tt) {
        return 5.0 * Math.sin(2.0 * tt) + 2.0 * Math.cos(0.5 * tt);
    }

    static double QinHyd(double tt) {
        return 0.8 * Math.sin(0.9 * tt) + 0.2 * Math.cos(0.3 * tt);
    }

    static double QdotTherm(double tt) {
        return 120.0 + 40.0 * Math.sin(0.15 * tt);
    }

    public static void main(String[] args) {
        double t0 = 0.0, tf = 12.0, dt = 1e-4;
        int N = (int) Math.round((tf - t0) / dt) + 1;

        double[] t = new double[N];
        for (int k = 0; k < N; k++) t[k] = t0 + k * dt;

        // ==========================================================
        // 1) Mechanical: m xdd + c xd + k x = F(t)
        // ==========================================================
        double m = 1.5, c = 0.8, ks = 12.0;
        double[] x = new double[N], v = new double[N];
        double[] PmIn = new double[N], PmDiss = new double[N], Em = new double[N];

        for (int k = 0; k < N - 1; k++) {
            double f = Fm(t[k]);
            double a = (f - c * v[k] - ks * x[k]) / m;

            v[k + 1] = v[k] + dt * a;
            x[k + 1] = x[k] + dt * v[k];

            PmIn[k] = f * v[k];
            PmDiss[k] = c * v[k] * v[k];
            Em[k] = 0.5 * m * v[k] * v[k] + 0.5 * ks * x[k] * x[k];
        }
        PmIn[N - 1] = Fm(t[N - 1]) * v[N - 1];
        PmDiss[N - 1] = c * v[N - 1] * v[N - 1];
        Em[N - 1] = 0.5 * m * v[N - 1] * v[N - 1] + 0.5 * ks * x[N - 1] * x[N - 1];
        double mechResidual = Em[N - 1] - Em[0] - (trapz(PmIn, t) - trapz(PmDiss, t));

        // ==========================================================
        // 2) Electrical: series RLC
        // ==========================================================
        double R = 2.0, L = 0.6, C = 0.25;
        double[] q = new double[N], i = new double[N];
        double[] PeIn = new double[N], PeDiss = new double[N], Ee = new double[N];

        for (int k = 0; k < N - 1; k++) {
            double vin = Vin(t[k]);
            double di = (vin - R * i[k] - q[k] / C) / L;
            double dq = i[k];

            i[k + 1] = i[k] + dt * di;
            q[k + 1] = q[k] + dt * dq;

            PeIn[k] = vin * i[k];
            PeDiss[k] = R * i[k] * i[k];
            Ee[k] = 0.5 * L * i[k] * i[k] + 0.5 * q[k] * q[k] / C;
        }
        PeIn[N - 1] = Vin(t[N - 1]) * i[N - 1];
        PeDiss[N - 1] = R * i[N - 1] * i[N - 1];
        Ee[N - 1] = 0.5 * L * i[N - 1] * i[N - 1] + 0.5 * q[N - 1] * q[N - 1] / C;
        double elecResidual = Ee[N - 1] - Ee[0] - (trapz(PeIn, t) - trapz(PeDiss, t));

        // ==========================================================
        // 3) Fluid: C_h dp/dt = q_in - q,   L_h dq/dt = p - R_h q
        // ==========================================================
        double Ch = 0.08, Lh = 0.15, Rh = 1.7;
        double[] p = new double[N], qf = new double[N];
        double[] PfIn = new double[N], PfDiss = new double[N], Ef = new double[N];

        for (int k = 0; k < N - 1; k++) {
            double qsrc = QinHyd(t[k]);
            double dp = (qsrc - qf[k]) / Ch;
            double dqf = (p[k] - Rh * qf[k]) / Lh;

            p[k + 1] = p[k] + dt * dp;
            qf[k + 1] = qf[k] + dt * dqf;

            PfIn[k] = p[k] * qsrc;
            PfDiss[k] = Rh * qf[k] * qf[k];
            Ef[k] = 0.5 * Ch * p[k] * p[k] + 0.5 * Lh * qf[k] * qf[k];
        }
        PfIn[N - 1] = p[N - 1] * QinHyd(t[N - 1]);
        PfDiss[N - 1] = Rh * qf[N - 1] * qf[N - 1];
        Ef[N - 1] = 0.5 * Ch * p[N - 1] * p[N - 1] + 0.5 * Lh * qf[N - 1] * qf[N - 1];
        double fluidResidual = Ef[N - 1] - Ef[0] - (trapz(PfIn, t) - trapz(PfDiss, t));

        // ==========================================================
        // 4) Thermal: C_th dT/dt = Q_in - (T - T_env)/R_th
        // ==========================================================
        double Cth = 600.0, Rth = 0.4, Tenv = 293.15;
        double[] T = new double[N], PtIn = new double[N], PtOut = new double[N], Et = new double[N];
        for (int k = 0; k < N; k++) T[k] = Tenv;

        for (int k = 0; k < N - 1; k++) {
            double qdotIn = QdotTherm(t[k]);
            double qdotOut = (T[k] - Tenv) / Rth;
            double dT = (qdotIn - qdotOut) / Cth;

            T[k + 1] = T[k] + dt * dT;

            PtIn[k] = qdotIn;
            PtOut[k] = qdotOut;
            Et[k] = Cth * (T[k] - Tenv);
        }
        PtIn[N - 1] = QdotTherm(t[N - 1]);
        PtOut[N - 1] = (T[N - 1] - Tenv) / Rth;
        Et[N - 1] = Cth * (T[N - 1] - Tenv);
        double thermResidual = Et[N - 1] - Et[0] - (trapz(PtIn, t) - trapz(PtOut, t));

        System.out.println("=== Energy Balance Audit (Java) ===");
        System.out.printf("Mechanical residual : %.6e%n", mechResidual);
        System.out.printf("Electrical residual : %.6e%n", elecResidual);
        System.out.printf("Fluid residual      : %.6e%n", fluidResidual);
        System.out.printf("Thermal residual    : %.6e%n%n", thermResidual);

        System.out.println("Final stored energies:");
        System.out.printf("E_mech(tf)  = %.6f%n", Em[N - 1]);
        System.out.printf("E_elec(tf)  = %.6f%n", Ee[N - 1]);
        System.out.printf("E_fluid(tf) = %.6f%n", Ef[N - 1]);
        System.out.printf("E_therm(tf) = %.6f%n", Et[N - 1]);
    }
}

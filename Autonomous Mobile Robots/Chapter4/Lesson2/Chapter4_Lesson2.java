/*
Chapter4_Lesson2.java
Autonomous Mobile Robots — Chapter 4 Lesson 2
Slip, Skid, and Terrain Interaction Models

This Java example provides a minimal 1D simulation (vehicle vx, wheel omega).
In Java robotics ecosystems, common libraries include:
  - EJML for linear algebra (not required here)
  - ROS 2 Java client libraries for integration (not used here)

Compile and run:
  javac Chapter4_Lesson2.java
  java Chapter4_Lesson2
*/

import java.util.Arrays;

public class Chapter4_Lesson2 {

    static double slipRatio(double vx, double omega, double R, double eps) {
        double denom = Math.max(Math.max(Math.abs(vx), Math.abs(R * omega)), eps);
        return (R * omega - vx) / denom;
    }

    static double hardGroundFx(double vx, double omega, double R, double Fz, double mu, double Ck) {
        double kappa = slipRatio(vx, omega, R, 1e-6);
        double FxLin = Ck * kappa;
        double cap = Math.max(mu * Fz, 0.0);
        return Math.max(-cap, Math.min(cap, FxLin));
    }

    static double softSoilFx(double kappa, double Fz, double muPeak, double kShape) {
        double s = (kappa > 0.0) ? 1.0 : (kappa < 0.0 ? -1.0 : 0.0);
        double muEff = muPeak * (1.0 - Math.exp(-kShape * Math.abs(kappa))) * s;
        return Fz * muEff;
    }

    static class SimResult {
        double[] t, vx, omega, kappa, Fx;
        SimResult(int N) {
            t = new double[N];
            vx = new double[N];
            omega = new double[N];
            kappa = new double[N];
            Fx = new double[N];
        }
    }

    static SimResult simulate1D(double Tcmd, String terrain, double tEnd, double dt) {
        // Parameters
        double m = 25.0;
        double R = 0.10;
        double Iw = 0.05;
        double bw = 0.02;
        double g = 9.81;

        double Fz = 0.25 * m * g;
        double Crr = 0.02;
        double Frr = Crr * Fz;
        double mu = 0.8;

        int N = (int)Math.round(tEnd / dt) + 1;
        SimResult out = new SimResult(N);

        for (int i = 0; i < N; i++) out.t[i] = i * dt;

        for (int i = 0; i < N - 1; i++) {
            double k = slipRatio(out.vx[i], out.omega[i], R, 1e-6);
            double Fx;

            if ("hard".equals(terrain)) {
                Fx = hardGroundFx(out.vx[i], out.omega[i], R, Fz, mu, 15000.0);
            } else if ("soft".equals(terrain)) {
                Fx = softSoilFx(k, Fz, 0.55, 10.0);
                Fx -= 0.015 * Fz * Math.signum(out.vx[i]);
            } else {
                throw new IllegalArgumentException("terrain must be 'hard' or 'soft'");
            }

            double ax = (Fx - Frr) / m;
            out.vx[i + 1] = out.vx[i] + dt * ax;

            double domega = (Tcmd - R * Fx - bw * out.omega[i]) / Iw;
            out.omega[i + 1] = out.omega[i] + dt * domega;

            out.Fx[i] = Fx;
            out.kappa[i] = k;
        }

        out.Fx[N - 1] = out.Fx[N - 2];
        out.kappa[N - 1] = out.kappa[N - 2];
        return out;
    }

    public static void main(String[] args) {
        SimResult hard = simulate1D(12.0, "hard", 4.0, 1e-3);
        SimResult soft = simulate1D(12.0, "soft", 4.0, 1e-3);

        int idx = hard.t.length - 1;
        System.out.println("Final (hard): vx=" + hard.vx[idx] + " m/s, kappa=" + hard.kappa[idx] + ", Fx=" + hard.Fx[idx] + " N");
        System.out.println("Final (soft): vx=" + soft.vx[idx] + " m/s, kappa=" + soft.kappa[idx] + ", Fx=" + soft.Fx[idx] + " N");

        System.out.println("If you want plots, write arrays to CSV and plot in your preferred tool.");
    }
}

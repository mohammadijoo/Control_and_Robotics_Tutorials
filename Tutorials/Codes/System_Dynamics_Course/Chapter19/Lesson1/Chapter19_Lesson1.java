// Chapter19_Lesson1.java
// 1D heat/diffusion and wave equations (explicit finite differences)

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Chapter19_Lesson1 {
    public static void main(String[] args) throws IOException {
        // Heat / diffusion: u_t = alpha u_xx
        double L = 1.0, alpha = 0.2;
        int Nx = 81;
        double dx = L / (Nx - 1);
        double r = 0.45;                     // r <= 0.5
        double dt = r * dx * dx / alpha;
        int Nt = (int)(0.25 / dt);

        double[] x = new double[Nx];
        double[] u = new double[Nx];
        double[] un = new double[Nx];
        for (int i = 0; i < Nx; i++) {
            x[i] = i * dx;
            u[i] = Math.sin(Math.PI * x[i]) + 0.2 * Math.sin(3.0 * Math.PI * x[i]);
        }
        u[0] = 0.0; u[Nx - 1] = 0.0;

        try (PrintWriter heatFile = new PrintWriter(new FileWriter("Chapter19_Lesson1_heat_java.csv"))) {
            heatFile.println("step,x,u");
            for (int n = 0; n <= Nt; n++) {
                for (int i = 0; i < Nx; i++) {
                    heatFile.println(n + "," + x[i] + "," + u[i]);
                    un[i] = u[i];
                }
                for (int i = 1; i < Nx - 1; i++) {
                    u[i] = un[i] + alpha * dt / (dx * dx) * (un[i + 1] - 2.0 * un[i] + un[i - 1]);
                }
                u[0] = 0.0; u[Nx - 1] = 0.0;
            }
        }

        // Wave: u_tt = c^2 u_xx
        double c = 1.0;
        int Nxw = 201;
        double dxw = L / (Nxw - 1);
        double s = 0.95;                     // s <= 1
        double dtw = s * dxw / c;
        int Ntw = (int)(1.0 / dtw);

        double[] xw = new double[Nxw];
        double[] up = new double[Nxw];
        double[] uc = new double[Nxw];
        double[] unext = new double[Nxw];
        for (int i = 0; i < Nxw; i++) {
            xw[i] = i * dxw;
            up[i] = Math.exp(-180.0 * (xw[i] - 0.35) * (xw[i] - 0.35));
        }
        up[0] = 0.0; up[Nxw - 1] = 0.0;

        System.arraycopy(up, 0, uc, 0, Nxw);
        double cfl2 = (c * dtw / dxw) * (c * dtw / dxw);
        for (int i = 1; i < Nxw - 1; i++) {
            uc[i] = up[i] + 0.5 * cfl2 * (up[i + 1] - 2.0 * up[i] + up[i - 1]);
        }
        uc[0] = 0.0; uc[Nxw - 1] = 0.0;

        try (PrintWriter waveFile = new PrintWriter(new FileWriter("Chapter19_Lesson1_wave_java.csv"))) {
            waveFile.println("step,x,u");
            for (int n = 0; n <= Ntw; n++) {
                for (int i = 0; i < Nxw; i++) {
                    waveFile.println(n + "," + xw[i] + "," + uc[i]);
                }
                for (int i = 1; i < Nxw - 1; i++) {
                    unext[i] = 2.0 * uc[i] - up[i] + cfl2 * (uc[i + 1] - 2.0 * uc[i] + uc[i - 1]);
                }
                unext[0] = 0.0; unext[Nxw - 1] = 0.0;
                double[] temp = up; up = uc; uc = unext; unext = temp;
            }
        }

        System.out.println("Saved CSV outputs for heat and wave equations.");
    }
}

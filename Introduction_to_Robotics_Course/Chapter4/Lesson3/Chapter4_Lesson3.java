
import java.util.*;

public class PowerWiringSignalFlow {
    public static void main(String[] args) {
        double Vb = 14.8, C_Ah = 5.0, eta = 0.9;

        // loads: (V, Iavg)        
        double[][] loads = new double[][]{
            {14.8, 2.0},
            {5.0,  1.2},
            {3.3,  0.5}
        };
        double Ptot = 0.0;
        for (double[] li : loads) Ptot += li[0] * li[1];

        double Eb = 3600.0 * Vb * C_Ah;
        double TrunH = eta * Eb / Ptot / 3600.0;

        System.out.println("Average power [W] = " + Ptot);
        System.out.println("Estimated runtime [h] = " + TrunH);

        double rho = 1.68e-8, L = 1.5, A = 1.0e-6, I = 5.0;
        double Rw = rho * L / A;
        double dV = I * Rw;
        System.out.println("Wire resistance [ohm] = " + Rw);
        System.out.println("Voltage drop [V] = " + dV);

        double R = 1e3, C = 0.1e-6;
        double fc = 1.0 / (2.0 * Math.PI * R * C);
        System.out.println("Cutoff frequency [Hz] = " + fc);
    }
}
      
public class RCLowPassSimulator {

    private final double R;
    private final double C;
    private final double RC;
    private final double h;  // sampling period
    private double vC;       // state: capacitor voltage

    public RCLowPassSimulator(double R, double C, double h) {
        this.R = R;
        this.C = C;
        this.RC = R * C;
        this.h = h;
        this.vC = 0.0;
    }

    public double step(double u) {
        // Forward Euler integration of dvC/dt = (u - vC)/(R*C)
        double dvC = (u - vC) / RC;
        vC += h * dvC;
        return vC;
    }

    public double getState() {
        return vC;
    }

    public static void main(String[] args) {
        double R = 10_000.0;
        double C = 1e-6;
        double h = 1e-4;
        double T = 0.1;
        int N = (int) (T / h);

        RCLowPassSimulator rc = new RCLowPassSimulator(R, C, h);

        double U0 = 1.0;
        for (int k = 0; k <= N; k += N / 10) {
            double vC = rc.step(U0);
            double t = (k + 1) * h;
            System.out.printf("t = %.6f s, vC = %.4f V%n", t, vC);
        }
    }
}

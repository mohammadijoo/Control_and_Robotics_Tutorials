
public class TradeoffDemo {
    public static void main(String[] args) {
        double Ts = 2.0, w0 = 400.0, eta = 0.85;
        double r = 0.6, L = 0.8, E = 70e9, I = 2.5e-7, g = 9.81;

        java.util.function.BiFunction<Double, Double, Double> jointTorque =
            (N, wj) -> {
                double wm = N*wj;
                double Tm = Ts*(1.0 - wm/w0);
                if (Tm < 0) Tm = 0;
                return eta*N*Tm;
            };

        java.util.function.BiFunction<Double, Double, Double> payloadLimit =
            (N, wj) -> jointTorque.apply(N, wj)/(r*g);

        double[] Ns = {20,40,80,120};
        for (double N : Ns) {
            double mp0 = payloadLimit.apply(N, 0.0);
            double wjmax = w0/N;
            System.out.println("N=" + N + " mp_max(0)=" + mp0 +
                               " kg, wj_max=" + wjmax + " rad/s");
        }

        double k = 3.0*E*I/Math.pow(L,3);
        System.out.println("k = " + k + " N/m");
    }
}
      
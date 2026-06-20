public class MimoCoupledSystem {

    public static void main(String[] args) {
        double t0 = 0.0;
        double tf = 10.0;
        double dt = 0.001;

        double y1 = 0.0;
        double y2 = 0.0;

        for (double t = t0; t <= tf; t += dt) {
            double u1 = 1.0;          // simple constant inputs
            double u2 = 0.5;

            double dy1 = -y1 + 0.5*y2 + u1;
            double dy2 = 0.3*y1 - y2 + u2;

            y1 += dt*dy1;
            y2 += dt*dy2;

            if (Math.abs(t - 1.0*Math.round(t)) < 1e-6) {
                System.out.printf("t=%.1f, y1=%.4f, y2=%.4f%n", t, y1, y2);
            }
        }
    }
}
      

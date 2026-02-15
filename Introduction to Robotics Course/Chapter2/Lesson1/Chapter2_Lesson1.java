
public class EscapementOscillator {
    public static void main(String[] args) {
        double I = 1.0, c = 0.08, k = 4.0;
        double J = 0.15, T = 1.0;  // impulse every T seconds
        double dt = 0.001, tf = 10.0;

        double phi = 0.2, dphi = 0.0;
        for (int step = 0; step * dt < tf; step++) {
            double t = step * dt;

            // impulse at multiples of T
            if (Math.abs(t - Math.round(t / T) * T) < dt / 2.0) {
                dphi += J / I;
            }

            double ddphi = -(c/I)*dphi - (k/I)*phi;
            dphi += ddphi * dt;
            phi += dphi * dt;

            if (step % 100 == 0) {
                System.out.println(t + " " + phi);
            }
        }
    }
}
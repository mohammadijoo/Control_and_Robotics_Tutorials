public class AdaptiveServo {
    public static void main(String[] args) {
        double dt = 0.001;
        double T = 5.0;
        int steps = (int)(T / dt);

        double aTrue = 2.0;
        double K = 10.0;
        double gamma = 5.0;

        double x = 0.0;
        double aHat = 0.0;

        for (int k = 0; k < steps; ++k) {
            double t = k * dt;
            double xRef = (t > 0.5) ? 1.0 : 0.0;
            double e = x - xRef;

            double uFb = -K * e;
            double uFf = aHat * x;
            double u = uFb + uFf;

            double xDot = u - aTrue * x;
            x += dt * xDot;

            aHat -= gamma * e * x * dt;
        }

        System.out.println("Final aHat = " + aHat);
    }
}
      

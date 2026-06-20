public class CascadeRK4 {

  // x1' = -2 x1 + u
  // x2' = -3 x2 + v, v = -x1 + u
  // y   = -x2 + v
  static double[] f(double[] x, double u) {
    double x1 = x[0];
    double x2 = x[1];
    double v  = -x1 + u;

    double dx1 = -2.0 * x1 + u;
    double dx2 = -3.0 * x2 + v;

    return new double[] { dx1, dx2 };
  }

  static double y(double[] x, double u) {
    double x1 = x[0];
    double x2 = x[1];
    double v  = -x1 + u;
    return -x2 + v;
  }

  public static void main(String[] args) {
    double[] x = new double[] { 0.0, 0.0 };
    double dt = 0.001;
    double T = 10.0;
    int N = (int)(T / dt);

    for (int k = 0; k <= N; k++) {
      double t = k * dt;
      double u = 1.0; // step input

      // RK4
      double[] k1 = f(x, u);

      double[] x2 = new double[] { x[0] + 0.5*dt*k1[0], x[1] + 0.5*dt*k1[1] };
      double[] k2 = f(x2, u);

      double[] x3 = new double[] { x[0] + 0.5*dt*k2[0], x[1] + 0.5*dt*k2[1] };
      double[] k3 = f(x3, u);

      double[] x4 = new double[] { x[0] + dt*k3[0], x[1] + dt*k3[1] };
      double[] k4 = f(x4, u);

      x[0] = x[0] + (dt/6.0) * (k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
      x[1] = x[1] + (dt/6.0) * (k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);

      if (k % 1000 == 0) {
        System.out.println("t=" + t + " x1=" + x[0] + " x2=" + x[1] + " y=" + y(x, u));
      }
    }
  }
}
      

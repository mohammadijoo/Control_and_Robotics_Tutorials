public class MimoEuler {
    public static void main(String[] args) {
        double[][] A = {
            { 0.0,  1.0 },
            { -4.0, -2.0 }
        };
        double[][] B = {
            { 1.0, 0.0 },
            { 0.0, 1.0 }
        };

        double[] x = { 0.0, 0.0 };  // state [x1, x2]
        double[] u = { 1.0, 0.0 };  // input [u1, u2]

        double dt = 0.001;
        int steps = 10000;

        for (int k = 0; k < steps; ++k) {
            double[] xdot = new double[2];
            xdot[0] = A[0][0] * x[0] + A[0][1] * x[1]
                    + B[0][0] * u[0] + B[0][1] * u[1];
            xdot[1] = A[1][0] * x[0] + A[1][1] * x[1]
                    + B[1][0] * u[0] + B[1][1] * u[1];

            x[0] += dt * xdot[0];
            x[1] += dt * xdot[1];

            if (k % 1000 == 0) {
                System.out.println("t=" + (k * dt)
                        + "  x1=" + x[0]
                        + "  x2=" + x[1]);
            }
        }
    }
}
      

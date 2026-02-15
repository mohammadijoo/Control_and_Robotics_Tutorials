public class SimpleClosedLoop {
    public static void main(String[] args) {
        double a = 1.0;
        double b = 2.0;
        double K = 3.0;
        double r0 = 1.0;

        double h = 0.001;
        double tFinal = 2.0;
        int N = (int) (tFinal / h);

        double[] y = new double[N + 1];
        double[] u = new double[N + 1];

        y[0] = 0.0;

        for (int k = 0; k < N; k++) {
            double e = r0 - y[k];
            u[k] = K * e;
            double dy = -(a + b * K) * y[k] + b * K * r0;
            y[k + 1] = y[k] + h * dy;
        }
        u[N] = u[N - 1];

        System.out.println("Final output y(T) = " + y[N]);
    }
}

/*
For robotics in Java, frameworks such as ROSJava provide interfaces to
ROS-based robot middleware, enabling Java-based supervisory controllers
to interact with lower-level C++ control loops.
*/

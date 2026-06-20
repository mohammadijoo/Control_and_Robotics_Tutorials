import org.ejml.simple.SimpleMatrix;

public class LinearODE_RK4 {
    static SimpleMatrix A_of_t(double t) {
        double a21 = -2.0 - 0.2*Math.sin(t);
        return new SimpleMatrix(new double[][] {
                {0.0, 1.0},
                {a21, -0.4}
        });
    }

    static SimpleMatrix b_of_t(double t) {
        return new SimpleMatrix(new double[][] {
                {0.0},
                {0.5*Math.cos(t)}
        });
    }

    static SimpleMatrix f(double t, SimpleMatrix x) {
        return A_of_t(t).mult(x).plus(b_of_t(t));
    }

    static SimpleMatrix rk4Step(double t, SimpleMatrix x, double h) {
        SimpleMatrix k1 = f(t, x);
        SimpleMatrix k2 = f(t + 0.5*h, x.plus(k1.scale(0.5*h)));
        SimpleMatrix k3 = f(t + 0.5*h, x.plus(k2.scale(0.5*h)));
        SimpleMatrix k4 = f(t + h,     x.plus(k3.scale(h)));
        return x.plus(k1.plus(k2.scale(2.0)).plus(k3.scale(2.0)).plus(k4).scale(h/6.0));
    }

    public static void main(String[] args) {
        double t0 = 0.0, tf = 10.0, h = 0.01;
        int N = (int)((tf - t0)/h);

        SimpleMatrix x = new SimpleMatrix(new double[][]{ {1.0},{0.0} });

        double t = t0;
        for (int k = 0; k < N; k++) {
            x = rk4Step(t, x, h);
            t += h;
        }

        System.out.println("Final x = [" + x.get(0,0) + ", " + x.get(1,0) + "]");
    }
}
      

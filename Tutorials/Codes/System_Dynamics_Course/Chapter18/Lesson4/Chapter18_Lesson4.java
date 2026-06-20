// Chapter18_Lesson4.java
// Bond graph simulation: Se - 1 - (I,R,C) for a mass-spring-damper
// Compile: javac Chapter18_Lesson4.java
// Run:     java Chapter18_Lesson4

import java.io.FileWriter;
import java.io.PrintWriter;

public class Chapter18_Lesson4 {
    static class Params {
        double m, b, C;
        Params(double m, double b, double C) { this.m = m; this.b = b; this.C = C; }
    }

    static double Se(double t) {
        return 2.0 * Math.sin(2.0 * Math.PI * 0.8 * t) + (t >= 1.0 ? 1.0 : 0.0);
    }

    static double[] rhs(double t, double[] x, Params p) {
        double q = x[0], mom = x[1];
        double v = mom / p.m;
        double eR = p.b * v;
        double eC = q / p.C;
        return new double[]{v, Se(t) - eR - eC};
    }

    static double[] rk4(double t, double[] x, double h, Params p) {
        double[] k1 = rhs(t, x, p);
        double[] x2 = new double[]{x[0] + 0.5*h*k1[0], x[1] + 0.5*h*k1[1]};
        double[] k2 = rhs(t + 0.5*h, x2, p);
        double[] x3 = new double[]{x[0] + 0.5*h*k2[0], x[1] + 0.5*h*k2[1]};
        double[] k3 = rhs(t + 0.5*h, x3, p);
        double[] x4 = new double[]{x[0] + h*k3[0], x[1] + h*k3[1]};
        double[] k4 = rhs(t + h, x4, p);

        return new double[]{
            x[0] + (h/6.0)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
            x[1] + (h/6.0)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        };
    }

    public static void main(String[] args) throws Exception {
        Params p = new Params(1.5, 1.2, 1.0/12.0);
        double h = 1e-3, t0 = 0.0, tf = 10.0;
        int N = (int)((tf - t0)/h) + 1;

        double[] t = new double[N], q = new double[N], mom = new double[N], v = new double[N];
        double[] H = new double[N], Ediss = new double[N], res = new double[N];

        q[0] = 0.10; mom[0] = 0.0; Ediss[0] = 0.0;
        for (int i = 0; i < N; i++) t[i] = t0 + i*h;

        for (int i = 0; i < N - 1; i++) {
            double[] xn = rk4(t[i], new double[]{q[i], mom[i]}, h, p);
            q[i+1] = xn[0];
            mom[i+1] = xn[1];
        }

        for (int i = 0; i < N; i++) {
            v[i] = mom[i] / p.m;
            H[i] = 0.5*mom[i]*mom[i]/p.m + 0.5*q[i]*q[i]/p.C;
        }

        for (int i = 1; i < N; i++) {
            Ediss[i] = Ediss[i-1] + h * (p.b * v[i-1] * v[i-1]);
        }

        double maxRes = 0.0;
        for (int i = 1; i < N - 1; i++) {
            double dH = (H[i+1] - H[i-1]) / (2*h);
            res[i] = Se(t[i]) * v[i] - p.b*v[i]*v[i] - dH;
            maxRes = Math.max(maxRes, Math.abs(res[i]));
        }
        System.out.println("Max |power residual| = " + maxRes);

        PrintWriter out = new PrintWriter(new FileWriter("Chapter18_Lesson4_java_output.csv"));
        out.println("t,q,v,H,Ediss,residual");
        for (int i = 0; i < N; i++) {
            out.printf(java.util.Locale.US, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n",
                    t[i], q[i], v[i], H[i], Ediss[i], res[i]);
        }
        out.close();
    }
}

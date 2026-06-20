// Chapter18_Lesson5.java
import java.io.FileWriter;
import java.io.PrintWriter;

public class Chapter18_Lesson5 {
    static class P { double L=0.35, Ra=1.8, J=0.02, b=0.08, k=4.0, Kt=0.22; }

    static double u(double t){ return (t < 0.8) ? 8.0 : ((t < 1.6) ? 3.0 : 0.0); }

    static double[] gradH(double[] x, P p){ return new double[]{x[0]/p.L, x[1]/p.J, p.k*x[2]}; }

    static double H(double[] x, P p){ return 0.5*x[0]*x[0]/p.L + 0.5*x[1]*x[1]/p.J + 0.5*p.k*x[2]*x[2]; }

    static double[] f(double t, double[] x, P p){
        double[] g = gradH(x,p); double i=g[0], w=g[1], ks=g[2];
        return new double[]{p.Kt*w - p.Ra*i + u(t), -p.Kt*i - ks - p.b*w, w};
    }

    static double[] add(double[] a, double[] b, double s){
        return new double[]{a[0]+s*b[0], a[1]+s*b[1], a[2]+s*b[2]};
    }

    static double[] rk4(double t, double[] x, double h, P p){
        double[] k1=f(t,x,p), k2=f(t+0.5*h, add(x,k1,0.5*h),p), k3=f(t+0.5*h, add(x,k2,0.5*h),p), k4=f(t+h, add(x,k3,h),p);
        return new double[]{
            x[0]+h*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6.0,
            x[1]+h*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6.0,
            x[2]+h*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6.0
        };
    }

    public static void main(String[] args) throws Exception {
        P p = new P(); double h=1e-3, tf=4.0; int N=(int)(tf/h)+1; double[] x={0,0,0};
        double intPow=0.0, prev=0.0, H0=H(x,p); boolean first=true;
        try (PrintWriter out = new PrintWriter(new FileWriter("Chapter18_Lesson5_java_results.csv"))) {
            out.println("t,phi,p,q,H,Pin,Pdiss");
            for(int n=0; n < N; n++){
                double t=n*h; double[] g=gradH(x,p); double i=g[0], w=g[1];
                double Pin=u(t)*i, Pdiss=p.Ra*i*i + p.b*w*w, Hx=H(x,p), s=Pin-Pdiss;
                if(!first) intPow += 0.5*h*(prev+s); first=false; prev=s;
                out.printf("%.6f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f%n", t, x[0], x[1], x[2], Hx, Pin, Pdiss);
                if(n < N-1) x = rk4(t,x,h,p);
            }
        }
        System.out.println("Energy residual = " + (H(x,p)-H0-intPow));
    }
}

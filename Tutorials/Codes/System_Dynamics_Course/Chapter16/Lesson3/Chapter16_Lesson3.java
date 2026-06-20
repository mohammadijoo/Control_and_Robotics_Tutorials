// Chapter16_Lesson3.java
public class Chapter16_Lesson3 {
    static class C {
        double re, im;
        C(double re, double im){ this.re = re; this.im = im; }
        C add(C o){ return new C(re + o.re, im + o.im); }
        C mul(C o){ return new C(re*o.re - im*o.im, re*o.im + im*o.re); }
        C mul(double s){ return new C(re*s, im*s); }
        C div(C o){
            double d = o.re*o.re + o.im*o.im;
            return new C((re*o.re + im*o.im)/d, (im*o.re - re*o.im)/d);
        }
        double abs(){ return Math.hypot(re, im); }
        double ang(){ return Math.atan2(im, re); }
    }

    static double[] impulse(double[] b, double[] a, int N){
        double[] x = new double[N], y = new double[N];
        x[0] = 1.0;
        for(int n=0; n<N; n++){
            for(int k=0; k<b.length; k++) if(n-k >= 0) y[n] += b[k]*x[n-k];
            for(int k=1; k<a.length; k++) if(n-k >= 0) y[n] -= a[k]*y[n-k];
            y[n] /= a[0];
        }
        return y;
    }

    static C Hejw(double[] b, double[] a, double w){
        C zinv = new C(Math.cos(w), -Math.sin(w));
        C num = new C(0,0), den = new C(0,0), p = new C(1,0);
        for (double bi : b){ num = num.add(p.mul(bi)); p = p.mul(zinv); }
        p = new C(1,0);
        for (double ai : a){ den = den.add(p.mul(ai)); p = p.mul(zinv); }
        return num.div(den);
    }

    public static void main(String[] args){
        double[] b = {0.2, 0.1}, a = {1.0, -1.5, 0.56};
        double[] h = impulse(b, a, 20);
        for(int i=0; i<10; i++) System.out.printf("h[%d]=%.6f%n", i, h[i]);
        for(int i=0; i<=4; i++){
            double w = Math.PI * i / 4.0;
            C H = Hejw(b, a, w);
            System.out.printf("w=%.3f |H|=%.6f phase=%.6f%n", w, H.abs(), H.ang());
        }
    }
}

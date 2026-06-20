public class CubicPTP {
    public static double cubicPTP(double q0, double qf, double T, double t){
        t = Math.max(0.0, Math.min(t, T));
        double Delta = qf - q0;
        double s = t / T;
        return q0 + 3.0*Delta*s*s - 2.0*Delta*s*s*s;
    }

    public static void main(String[] args){
        double q0=0.2, qf=1.0, T=2.0;
        for(int i=0;i<5;i++){
            double t = i*(T/4.0);
            System.out.println(cubicPTP(q0,qf,T,t));
        }
    }
}
      
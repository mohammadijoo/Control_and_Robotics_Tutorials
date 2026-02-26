public class MassSpringDamper {
    public static void main(String[] args){
        double m=1.0, b=0.6, k=4.0;
        double h=0.2, T=5.0;
        int N = (int)(T/h);

        double q=0.0, dq=0.0;
        for(int i=0;i<N;i++){
            double u = 1.0;
            double ddq = (u - b*dq - k*q)/m;
            dq += h*ddq;
            q  += h*dq;
        }
        System.out.println("final q = " + q);
    }
}
      

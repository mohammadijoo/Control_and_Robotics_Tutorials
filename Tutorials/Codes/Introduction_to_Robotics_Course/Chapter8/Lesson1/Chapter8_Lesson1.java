import java.util.Random;

public class LowPassDemo {
    public static void main(String[] args) {
        double Ts = 0.01;
        double tau = 0.1;
        double alpha = Ts / (tau + Ts);

        int N = 500;
        double[] yq = new double[N];
        double[] shat = new double[N];
        Random rng = new Random(0);

        for(int k=0;k<N;k++){
            double t = k*Ts;
            double xTrue = Math.sin(2*Math.PI*1.0*t);
            double y = xTrue + 0.2*rng.nextGaussian();

            // quantize to 8-bit over [-1.5,1.5]
            double ymin = -1.5, ymax = 1.5;
            int b = 8;
            double Delta = (ymax-ymin)/Math.pow(2,b);
            yq[k] = ymin + Delta*Math.round((y-ymin)/Delta);

            if(k>0){
                shat[k] = (1-alpha)*shat[k-1] + alpha*yq[k];
            }
        }
        System.out.println("Final filtered value = " + shat[N-1]);
    }
}

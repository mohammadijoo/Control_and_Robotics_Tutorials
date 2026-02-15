import java.util.Random;

public class SensorModel {
    public static void main(String[] args){
        double fs = 100.0, Ts = 1.0/fs;
        int N = 200;
        double[] x = new double[N];
        double[] b = new double[N];
        double[] yq = new double[N];

        Random rng = new Random(0);

        double sigmaN = 0.05;
        double sigmaB = 1e-4;

        int Bbits = 10;
        double ymin = -2.0, ymax = 2.0;
        double Delta = (ymax - ymin) / (1 << Bbits);

        for(int k=0; k<N; k++){
            double t = k*Ts;
            x[k] = Math.sin(2*Math.PI*3*t);
            if(k==0) b[k]=0; 
            else b[k] = b[k-1] + sigmaB*rng.nextGaussian();

            double y = x[k] + b[k] + sigmaN*rng.nextGaussian();
            yq[k] = Delta * Math.round(y/Delta);
        }

        // Bias estimate from initial window
        double bHat = 0;
        for(int k=0; k<20; k++) bHat += yq[k];
        bHat /= 20.0;

        System.out.println("Delta=" + Delta + ", bHat=" + bHat);
    }
}

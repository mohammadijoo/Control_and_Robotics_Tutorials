import org.ejml.simple.SimpleMatrix;
import java.util.Random;

public class FrictionLS {
    public static void main(String[] args){
        double Ts = 0.01;
        double mNom = 2.0;
        int N = 2000;
        double[] u = new double[N];
        double[] v = new double[N];
        Random rnd = new Random(0);

        for(int k=0;k<N;k++){
            u[k] = 2.0*Math.sin(0.5*k*Ts);
            v[k] = 0.5*Math.sin(0.5*k*Ts) + 0.02*(rnd.nextDouble()-0.5);
        }

        SimpleMatrix Phi = new SimpleMatrix(N-1,1);
        SimpleMatrix y   = new SimpleMatrix(N-1,1);

        for(int k=0;k<N-1;k++){
            double dv = (v[k+1]-v[k])/Ts;
            y.set(k,0, dv - u[k]/mNom);
            Phi.set(k,0, -v[k]/mNom);
        }

        // b_hat = (Phi^T Phi)^-1 Phi^T y
        double bHat = Phi.transpose().mult(Phi).invert().mult(Phi.transpose()).mult(y).get(0,0);
        System.out.println("Estimated b = " + bHat);
    }
}
      

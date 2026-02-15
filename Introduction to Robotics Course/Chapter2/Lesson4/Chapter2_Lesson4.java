import java.util.Random;

public class LinearGD {
    public static void main(String[] args) {
        int N = 200;
        double[][] X = new double[N][2];
        double[] y = new double[N];
        Random rng = new Random(1);

        double[] wTrue = {1.2, -0.8};
        double bTrue = -0.1;
        for(int i=0; i<N; i++){
            X[i][0] = rng.nextGaussian();
            X[i][1] = rng.nextGaussian();
            y[i] = wTrue[0]*X[i][0] + wTrue[1]*X[i][1] + bTrue
                   + 0.1*rng.nextGaussian();
        }

        double[] w = {0.0, 0.0};
        double b = 0.0, eta = 0.05;
        for(int k=0; k<3000; k++){
            double gw0=0, gw1=0, gb=0;
            for(int i=0; i<N; i++){
                double pred = w[0]*X[i][0] + w[1]*X[i][1] + b;
                double err = pred - y[i];
                gw0 += err*X[i][0];
                gw1 += err*X[i][1];
                gb  += err;
            }
            gw0 *= (2.0/N); gw1 *= (2.0/N); gb *= (2.0/N);
            w[0] -= eta*gw0; w[1] -= eta*gw1; b -= eta*gb;
        }

        System.out.println("learned w0=" + w[0] + " w1=" + w[1] + " b=" + b);
    }
}
      
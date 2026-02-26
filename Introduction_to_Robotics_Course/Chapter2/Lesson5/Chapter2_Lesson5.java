import java.util.Arrays;

public class ConsensusDemo {
    public static void main(String[] args) {
        int n = 4;
        double[][] L = { // Laplacian of a connected graph
            { 2, -1, -1,  0},
            {-1,  2,  0, -1},
            {-1,  0,  2, -1},
            { 0, -1, -1,  2}
        };

        double[] x = {1.0, 0.0, 2.0, -1.0};
        double dt = 0.05;

        for(int t=0; t<200; t++){
            double[] dx = new double[n];
            for(int i=0;i<n;i++){
                for(int j=0;j<n;j++){
                    dx[i] += -L[i][j]*x[j];
                }
            }
            for(int i=0;i<n;i++) x[i] += dt*dx[i];
        }
        System.out.println("Consensus state: " + Arrays.toString(x));
    }
}
      
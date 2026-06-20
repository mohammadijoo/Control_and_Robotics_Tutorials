import java.util.*;

public class FieldRobotClassifier {
  static double mahalanobis(double[] f, double[] mu, double[][] SigmaInv){
    double[] d = new double[f.length];
    for(int i=0;i<f.length;i++) d[i]=f[i]-mu[i];
    double sum=0;
    for(int i=0;i<f.length;i++)
      for(int j=0;j<f.length;j++)
        sum += d[i]*SigmaInv[i][j]*d[j];
    return Math.sqrt(sum);
  }

  public static void main(String[] args){
    // Example with 3 features: density, gravity, latency
    double[] f = {1.2, 9.81, 0.05}; // candidate robot
    Map<String,double[]> centroids = new HashMap<>();
    centroids.put("Aerial", new double[]{1.2, 9.81, 0.01});
    centroids.put("Underwater", new double[]{1000, 9.81, 0.2});
    centroids.put("Space", new double[]{0.0, 0.0, 1.0});

    double[][] SigmaInv = {
      {1.0,0,0},
      {0,1.0,0},
      {0,0,1.0}
    };

    String best=null; double bestD=1e9;
    for(String c: centroids.keySet()){
      double d = mahalanobis(f, centroids.get(c), SigmaInv);
      if(d < bestD){ bestD=d; best=c; }
    }
    System.out.println("classified as: " + best);
  }
}
      
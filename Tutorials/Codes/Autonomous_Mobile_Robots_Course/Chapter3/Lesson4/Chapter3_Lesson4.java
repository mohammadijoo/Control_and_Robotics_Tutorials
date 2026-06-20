\
/*
Chapter 3 — Lesson 4: Motion Primitives for Ground Vehicles (conceptual use)
Compile:
  javac Chapter3_Lesson4.java
Run:
  java Chapter3_Lesson4
*/
import java.util.Locale;

public class Chapter3_Lesson4 {
  static double[] endpoint(double v, double kappa, double T){
    double dth = v*kappa*T;
    if (Math.abs(kappa) < 1e-12) return new double[]{v*T, 0.0, dth};
    return new double[]{Math.sin(dth)/kappa, (1.0-Math.cos(dth))/kappa, dth};
  }
  public static void main(String[] args){
    double[] ep = endpoint(0.6, 0.4, 1.2);
    System.out.printf(Locale.US, "dx=%.6f dy=%.6f dtheta=%.6f%n", ep[0], ep[1], ep[2]);
  }
}

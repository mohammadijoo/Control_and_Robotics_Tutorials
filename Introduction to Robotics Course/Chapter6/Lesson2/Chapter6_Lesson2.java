import java.util.*;

public class TransmissionMath {
    static double eqRatio(List<double[]> stages){
        double N = 1.0;
        for(double[] s : stages){
            double z1 = s[0], z2 = s[1];
            N *= (z2 / z1);
        }
        return N;
    }

    static double backlashTorque(double thetaM, double thetaL, double N,
                                 double b, double kt){
        double dtheta = thetaM / N - thetaL;
        if(Math.abs(dtheta) <= b/2.0) return 0.0;
        if(dtheta > b/2.0) return kt * (dtheta - b/2.0);
        return kt * (dtheta + b/2.0);
    }

    public static void main(String[] args){
        List<double[]> stages = Arrays.asList(new double[]{20,80}, new double[]{18,54});
        double N = eqRatio(stages);
        System.out.println("N_eq=" + N);

        double tauL = backlashTorque(N*0.03, 0.0, N, 0.02, 150.0);
        System.out.println("tau_l=" + tauL);
    }
}

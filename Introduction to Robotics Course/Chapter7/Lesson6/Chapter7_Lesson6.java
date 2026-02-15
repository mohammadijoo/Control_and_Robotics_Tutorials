import java.util.*;
import static java.lang.Math.*;

class Sensor {
    double rmin, rmax, fs, Delta, sigmaV, cost;
    Sensor(double rmin,double rmax,double fs,double Delta,double sigmaV,double cost){
        this.rmin=rmin; this.rmax=rmax; this.fs=fs; this.Delta=Delta; this.sigmaV=sigmaV; this.cost=cost;
    }
}

public class SensorSelection {
    static double[] normalize(double[] v){
        double mn=Arrays.stream(v).min().getAsDouble();
        double mx=Arrays.stream(v).max().getAsDouble();
        double[] out=new double[v.length];
        for(int i=0;i<v.length;i++) out[i]=(v[i]-mn)/(mx-mn+1e-12);
        return out;
    }

    public static void main(String[] args){
        List<Sensor> S=List.of(
            new Sensor(0.0,5.0,200.0,0.01,0.02,50.0),
            new Sensor(0.1,10.0,60.0,0.05,0.05,20.0),
            new Sensor(0.0,8.0,120.0,0.02,0.03,35.0)
        );

        double xMinReq=0.0, xMaxReq=6.0, B=40.0, deltaX=0.03, epsMax=0.05;

        List<Integer> feasibleIdx=new ArrayList<>();
        List<Double> sigmaTot=new ArrayList<>();
        List<Double> latency=new ArrayList<>();
        List<Double> cost=new ArrayList<>();

        for(int i=0;i<S.size();i++){
            Sensor s=S.get(i);
            boolean ok = (s.rmin <= xMinReq && s.rmax >= xMaxReq);
            ok = ok && (s.fs >= 2*B);
            ok = ok && (s.Delta <= deltaX);
            double stot = sqrt(s.sigmaV*s.sigmaV + s.Delta*s.Delta/12.0);
            ok = ok && (stot <= epsMax);

            if(ok){
                feasibleIdx.add(i);
                sigmaTot.add(stot);
                latency.add(1.0/s.fs);
                cost.add(s.cost);
            }
        }

        if(feasibleIdx.isEmpty()){
            System.out.println("No feasible sensors");
            return;
        }

        double[] a1=sigmaTot.stream().mapToDouble(d->d).toArray();
        double[] a2=latency.stream().mapToDouble(d->d).toArray();
        double[] a3=cost.stream().mapToDouble(d->d).toArray();
        a1=normalize(a1); a2=normalize(a2); a3=normalize(a3);

        double[] w={0.5,0.3,0.2};
        double bestScore=Double.POSITIVE_INFINITY;
        int bestGlobal=-1;

        for(int k=0;k<feasibleIdx.size();k++){
            double score=w[0]*a1[k]+w[1]*a2[k]+w[2]*a3[k];
            if(score<bestScore){
                bestScore=score;
                bestGlobal=feasibleIdx.get(k);
            }
        }

        System.out.println("Best sensor index: "+bestGlobal);
    }
}

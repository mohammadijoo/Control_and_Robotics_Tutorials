import org.ejml.simple.SimpleMatrix;
import java.util.*;

public class AppDrivenClassification {

    static boolean dominates(SimpleMatrix ci, SimpleMatrix cj){
        boolean geAll = true;
        boolean gAny = false;
        for(int j=0;j<ci.numCols();j++){
            double a = ci.get(0,j), b = cj.get(0,j);
            if(a < b) geAll = false;
            if(a > b) gAny = true;
        }
        return geAll && gAny;
    }

    public static void main(String[] args){
        String[] classes = {"IndustrialArm","MobileBase","MobileManipulator","Humanoid"};

        SimpleMatrix r = new SimpleMatrix(1,4,true, new double[]{0.7,0.8,0.5,0.9});

        SimpleMatrix C = new SimpleMatrix(4,4,true, new double[]{
            0.95,0.20,0.90,0.60,
            0.30,0.90,0.40,0.70,
            0.80,0.85,0.70,0.85,
            0.75,0.70,0.55,0.90
        });

        List<Integer> feasible = new ArrayList<>();
        for(int i=0;i<C.numRows();i++){
            SimpleMatrix ci = C.extractVector(true,i);
            boolean ok = true;
            for(int j=0;j<4;j++){
                if(ci.get(0,j) < r.get(0,j)) ok = false;
            }
            if(ok) feasible.add(i);
        }

        System.out.println("Feasible: " + feasible.stream().map(i->classes[i]).toList());

        List<Integer> pareto = new ArrayList<>();
        for(int i: feasible){
            boolean dom = false;
            for(int j: feasible){
                if(i==j) continue;
                if(dominates(C.extractVector(true,j), C.extractVector(true,i))){
                    dom = true; break;
                }
            }
            if(!dom) pareto.add(i);
        }

        System.out.println("Pareto: " + pareto.stream().map(i->classes[i]).toList());
    }
}
      
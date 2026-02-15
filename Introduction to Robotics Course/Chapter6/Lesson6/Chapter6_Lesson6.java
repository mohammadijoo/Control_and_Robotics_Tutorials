import java.util.*;

class Motor {
    String name;
    double tauCont, tauPeak, omegaMax, Jm, mass, cost, eta;
    Motor(String n,double tc,double tp,double om,double jm,double ms,double c,double e){
        name=n; tauCont=tc; tauPeak=tp; omegaMax=om; Jm=jm; mass=ms; cost=c; eta=e;
    }
}

public class ActuatorSelect {
    public static void main(String[] args){
        double tauLPeak=35.0, omegaLPeak=4.0, alphaLPeak=12.0, etaG=0.92;
        int[] gears = {5,8,12,16};

        List<Motor> catalog = Arrays.asList(
            new Motor("BLDC-A",12,40,80,0.0008,1.2,300,0.90),
            new Motor("Servo-B",20,60,40,0.0016,2.2,550,0.85),
            new Motor("Hydro-C",50,120,25,0.0030,6.0,900,0.70)
        );

        double[] samples = {0,10,30,10,0};
        double meanSq=0;
        for(double s: samples) meanSq += s*s;
        double tauRmsReq = Math.sqrt(meanSq/samples.length);

        for(Motor m: catalog){
            for(int g: gears){
                double omegaMPeak = g*omegaLPeak;
                double tauMPeak = (tauLPeak/(g*etaG)) + g*m.Jm*alphaLPeak;
                if(tauMPeak <= m.tauPeak && omegaMPeak <= m.omegaMax && tauRmsReq <= m.tauCont){
                    System.out.println("Feasible: "+m.name+" gear "+g);
                }
            }
        }
    }
}
      

import java.util.Random;

interface Sensor {
    double read(double p, double v);
}

class ProprioVelocitySensor implements Sensor {
    private final double bias, sigma;
    private final Random rng = new Random();
    public ProprioVelocitySensor(double bias, double sigma){
        this.bias = bias; this.sigma = sigma;
    }
    public double read(double p, double v){
        return v + bias + sigma * rng.nextGaussian();
    }
}

class ExteroPositionSensor implements Sensor {
    private final double sigma;
    private final Random rng = new Random();
    public ExteroPositionSensor(double sigma){
        this.sigma = sigma;
    }
    public double read(double p, double v){
        return p + sigma * rng.nextGaussian();
    }
}

public class SensorDemo {
    public static void main(String[] args){
        Sensor enc = new ProprioVelocitySensor(0.05, 0.02);
        Sensor rng = new ExteroPositionSensor(0.05);

        double p=0.0, v=1.0, Ts=0.1, pHat=0.0;
        for(int k=0;k<200;k++){
            p += Ts*v;
            double yP = enc.read(p,v);
            double yE = rng.read(p,v);
            pHat += Ts*yP;
            if(k % 50 == 0){
                System.out.println("k="+k+
                    " true p="+p+
                    " proprio pHat="+pHat+
                    " extero yE="+yE);
            }
        }
    }
}

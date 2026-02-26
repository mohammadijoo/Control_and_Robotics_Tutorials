import java.util.*;
import java.util.function.Consumer;

// L1 bus
class Bus {
    private Map<String, List<Consumer<Double>>> subs = new HashMap<>();
    void publish(String topic, double msg){
        subs.getOrDefault(topic, List.of()).forEach(cb -> cb.accept(msg));
    }
    void subscribe(String topic, Consumer<Double> cb){
        subs.computeIfAbsent(topic, k -> new ArrayList<>()).add(cb);
    }
}

// L0 driver
class EncoderDriver {
    int cpr = 4096;
    int counts = 0;
    int readCounts(){ return counts; }
    void writePWM(double pwm){ counts += (int)(0.1*pwm); }
}

// L2 controller
class PositionController {
    Bus bus; double kp=0.2; double ref=0.0;
    PositionController(Bus b){
        bus=b;
        bus.subscribe("ref", r -> ref=r);
        bus.subscribe("pos", this::onPos);
    }
    void onPos(double pos){
        double u = kp*(ref-pos);
        bus.publish("cmd_pwm", u);
    }
}

public class Main {
    public static void main(String[] args){
        EncoderDriver drv = new EncoderDriver();
        Bus bus = new Bus();
        new PositionController(bus);
        bus.subscribe("cmd_pwm", drv::writePWM);

        double t=0, dt=0.01;
        for(int k=0;k<400;k++){
            double ref = (t>1.0)?10.0:0.0;
            bus.publish("ref", ref);

            double pos = drv.readCounts()/(double)drv.cpr * 2*Math.PI;
            bus.publish("pos", pos);

            t += dt;
        }
    }
}
      

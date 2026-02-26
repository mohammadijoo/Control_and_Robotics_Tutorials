import java.util.concurrent.atomic.AtomicReference;

// ---------------- Middleware Abstraction ----------------
class Topic<T> {
    private final AtomicReference<T> msg = new AtomicReference<>();
    private volatile long tNano = 0;

    public void publish(T m){
        msg.set(m);
        tNano = System.nanoTime();
    }

    public T latest(){
        return msg.get();
    }

    public double ageSeconds(){
        return (System.nanoTime() - tNano) * 1e-9;
    }
}

// ---------------- Application Layer ----------------
public class RobotApp {
    static Topic<Double> imuTopic = new Topic<>();

    public static void main(String[] args) throws Exception {
        double K = 0.5;

        while(true){
            Double y = imuTopic.latest();
            if(y != null){
                double u = -K * y;
                System.out.println("[APP] y=" + y + " u=" + u
                    + " age=" + imuTopic.ageSeconds() + "s");
            }
            Thread.sleep(10);
        }
    }
}
      

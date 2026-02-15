// Gradle/Maven: org.zeromq:jeromq
import org.zeromq.ZMQ;
import org.json.JSONObject;

public class PubSub {
    static void publisher(double rateHz){
        ZMQ.Context ctx = ZMQ.context(1);
        ZMQ.Socket pub = ctx.socket(ZMQ.PUB);
        pub.bind("tcp://*:5556");
        long k = 0;
        long Tms = (long)(1000.0/rateHz);
        while(true){
            JSONObject msg = new JSONObject();
            msg.put("seq", k);
            msg.put("t_pub", System.currentTimeMillis()/1000.0);
            pub.send("imu " + msg.toString());
            k++;
            try { Thread.sleep(Tms); } catch(Exception e){}
        }
    }

    static void subscriber(double muHz){
        ZMQ.Context ctx = ZMQ.context(1);
        ZMQ.Socket sub = ctx.socket(ZMQ.SUB);
        sub.connect("tcp://localhost:5556");
        sub.subscribe("imu");
        long Tms = (long)(1000.0/muHz);
        while(true){
            String s = sub.recvStr();
            String payload = s.split(" ",2)[1];
            JSONObject msg = new JSONObject(payload);
            double age = System.currentTimeMillis()/1000.0 - msg.getDouble("t_pub");
            System.out.println("seq=" + msg.getLong("seq") + " age=" + age);
            try { Thread.sleep(Tms); } catch(Exception e){}
        }
    }

    public static void main(String[] args){
        new Thread(() -> publisher(50.0)).start();
        subscriber(40.0);
    }
}
      

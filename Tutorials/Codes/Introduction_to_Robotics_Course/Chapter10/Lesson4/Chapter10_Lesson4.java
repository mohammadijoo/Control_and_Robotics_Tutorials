import java.io.*;
import java.net.*;
import java.util.Arrays;

public class OffboardClient {
    public static void main(String[] args) throws Exception {
        String host = "192.168.1.10";
        int port = 9100;

        byte[] sensorPacket = new byte[5000]; // example payload
        Arrays.fill(sensorPacket, (byte)1);

        try (Socket socket = new Socket(host, port)) {
            DataOutputStream out = new DataOutputStream(socket.getOutputStream());
            DataInputStream in   = new DataInputStream(socket.getInputStream());

            long t0 = System.nanoTime();

            out.writeInt(sensorPacket.length);
            out.write(sensorPacket);
            out.flush();

            int replyLen = in.readInt();
            byte[] reply = new byte[replyLen];
            in.readFully(reply);

            long t1 = System.nanoTime();
            double rtt = (t1 - t0) * 1e-9;

            System.out.println("Round-trip time: " + rtt + " s");
        }
    }
}

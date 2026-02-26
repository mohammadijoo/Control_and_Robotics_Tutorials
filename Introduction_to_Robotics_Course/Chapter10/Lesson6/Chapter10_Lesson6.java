import com.fazecast.jSerialComm.SerialPort;
import java.nio.ByteBuffer;
import java.util.ArrayList;

public class BringUpClient {
    static int crc16(byte[] data) {
        int poly = 0x1021, crc = 0xFFFF;
        for (byte b : data) {
            crc ^= (b & 0xFF) << 8;
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x8000) != 0) crc = ((crc << 1) ^ poly) & 0xFFFF;
                else crc = (crc << 1) & 0xFFFF;
            }
        }
        return crc;
    }

    static byte[] makePkt(int cmd, byte[] payload) {
        int len = payload.length;
        byte[] hdr = new byte[]{(byte)0xAA, (byte)cmd, (byte)len};
        byte[] body = new byte[3 + len];
        System.arraycopy(hdr, 0, body, 0, 3);
        System.arraycopy(payload, 0, body, 3, len);

        int crc = crc16(body);
        ByteBuffer bb = ByteBuffer.allocate(body.length + 2);
        bb.put(body);
        bb.putShort((short)crc);
        return bb.array();
    }

    public static void main(String[] args) throws Exception {
        SerialPort sp = SerialPort.getCommPort("/dev/ttyUSB0");
        sp.setBaudRate(115200);
        if (!sp.openPort()) throw new RuntimeException("Port open failed");

        // Ping
        sp.writeBytes(makePkt(0x01, new byte[]{}), 5);

        ArrayList<Float> xs = new ArrayList<>();
        for (int i = 0; i < 200; i++) {
            sp.writeBytes(makePkt(0x10, new byte[]{}), 5);
            byte[] buf = new byte[3 + 4 + 2];
            sp.readBytes(buf, buf.length);

            // Parse float payload (big-endian)
            float x = ByteBuffer.wrap(buf, 3, 4).getFloat();
            xs.add(x);
            Thread.sleep(10);
        }

        double mean = xs.stream().mapToDouble(f -> f).average().orElse(0.0);
        System.out.println("mean=" + mean);

        sp.closePort();
    }
}

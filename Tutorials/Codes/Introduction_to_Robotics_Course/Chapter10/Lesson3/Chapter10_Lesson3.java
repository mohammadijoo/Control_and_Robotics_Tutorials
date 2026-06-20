import com.fazecast.jSerialComm.SerialPort;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class UartDemo {
    public static void main(String[] args) {
        SerialPort port = SerialPort.getCommPort("/dev/ttyUSB0");
        port.setBaudRate(115200);
        port.openPort();

        float v = 0.8f;
        ByteBuffer bb = ByteBuffer.allocate(5).order(ByteOrder.LITTLE_ENDIAN);
        bb.put((byte)0xAA);
        bb.putFloat(v);
        port.writeBytes(bb.array(), 5);

        byte[] rx = new byte[5];
        int n = port.readBytes(rx, 5);
        if (n == 5 && rx[0] == (byte)0x55) {
            float echo = ByteBuffer.wrap(rx, 1, 4).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            System.out.println("Echo: " + echo);
        }
        port.closePort();
    }
}

import com.fazecast.jSerialComm.SerialPort;

public class SensorConsole {

    public static void main(String[] args) {
        SerialPort port = SerialPort.getCommPort("/dev/ttyUSB0");
        port.setBaudRate(115200);
        port.setNumDataBits(8);
        port.setNumStopBits(SerialPort.ONE_STOP_BIT);
        port.setParity(SerialPort.NO_PARITY);

        if (!port.openPort()) {
            System.err.println("Failed to open serial port.");
            return;
        }

        try {
            byte[] buffer = new byte[256];
            StringBuilder lineBuilder = new StringBuilder();

            while (true) {
                int n = port.readBytes(buffer, buffer.length);
                if (n <= 0) {
                    continue;
                }
                for (int i = 0; i != n; ++i) {
                    char c = (char) buffer[i];
                    if (c == '\n') {
                        String line = lineBuilder.toString().trim();
                        lineBuilder.setLength(0);
                        handleLine(line);
                    } else {
                        lineBuilder.append(c);
                    }
                }
            }
        } finally {
            port.closePort();
        }
    }

    private static void handleLine(String line) {
        // Suppose firmware sends lines like "temp:23.5"
        if (!line.startsWith("temp:")) {
            return;
        }
        try {
            double value = Double.parseDouble(line.substring(5));
            System.out.println("Temperature [degC]: " + value);
        } catch (NumberFormatException ex) {
            // ignore malformed line
        }
    }
}
      

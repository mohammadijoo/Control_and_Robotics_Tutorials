import java.util.concurrent.atomic.AtomicBoolean;

public class PowerMonitor implements Runnable {
    private final AtomicBoolean shedRequested = new AtomicBoolean(false);

    // thresholds
    private final double vMin = 20.0;
    private final double vWarn = 21.0;

    public boolean isShedRequested() { return shedRequested.get(); }

    @Override
    public void run() {
        while(true) {
            double vbus = readVbus();  // via I2C/SPI bridge or USB sensor
            double ibus = readIbus();

            if(vbus < vMin) {
                // hard interlock should already have tripped in MCU
                log("Brownout: request safe stop");
                requestSafeStop();
            } else if(vbus < vWarn) {
                shedRequested.set(true);
                log("Low voltage: requesting load shedding");
            } else {
                shedRequested.set(false);
            }

            sleepMs(20); // 50 Hz monitor
        }
    }

    private void requestSafeStop(){ /* send command over CAN/UART */ }
    private double readVbus(){ return 24.0; }
    private double readIbus(){ return 3.0; }
    private void log(String s){ System.out.println(s); }
    private void sleepMs(int ms){
        try { Thread.sleep(ms); } catch(InterruptedException e){ }
    }
}

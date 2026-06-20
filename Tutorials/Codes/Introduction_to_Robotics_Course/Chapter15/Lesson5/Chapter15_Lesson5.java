import java.time.Instant;
import java.util.List;

class LogEntry {
    public final Instant timestamp;
    public final int group;     // 0 or 1
    public final double cost;   // harm metric
    public final String nodeId; // software component
    public final String version;

    public LogEntry(Instant ts, int group, double cost,
                    String nodeId, String version) {
        this.timestamp = ts;
        this.group = group;
        this.cost = cost;
        this.nodeId = nodeId;
        this.version = version;
    }
}

class Audit {

    public static double groupRisk(List<LogEntry> log, int g) {
        double sum = 0.0;
        int count = 0;
        for (LogEntry e : log) {
            if (e.group == g) {
                sum += e.cost;
                count++;
            }
        }
        if (count == 0) {
            throw new IllegalArgumentException("No samples for group " + g);
        }
        return sum / (double) count;
    }

    public static boolean checkDisparity(List<LogEntry> log, double eps) {
        double r0 = groupRisk(log, 0);
        double r1 = groupRisk(log, 1);
        double d = Math.abs(r1 - r0);
        return d <= eps;
    }
}
      

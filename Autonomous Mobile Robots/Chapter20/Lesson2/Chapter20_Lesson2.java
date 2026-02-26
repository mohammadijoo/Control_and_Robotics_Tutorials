// Chapter20_Lesson2.java
// Java implementation: sensor timing and middleware-load simulation for a selected sensor suite.
// Run: javac Chapter20_Lesson2.java && java Chapter20_Lesson2

import java.util.*;

public class Chapter20_Lesson2 {
    static class Sensor {
        String name;
        double rateHz;
        double processingMs;
        double jitterStdMs;
        double dropProb;
        int priority;

        Sensor(String name, double rateHz, double processingMs, double jitterStdMs, double dropProb, int priority) {
            this.name = name;
            this.rateHz = rateHz;
            this.processingMs = processingMs;
            this.jitterStdMs = jitterStdMs;
            this.dropProb = dropProb;
            this.priority = priority;
        }

        double periodMs() { return 1000.0 / rateHz; }
    }

    static class Event {
        Sensor sensor;
        double idealTimeMs;
        double actualTimeMs;
        boolean dropped;

        Event(Sensor sensor, double idealTimeMs, double actualTimeMs, boolean dropped) {
            this.sensor = sensor;
            this.idealTimeMs = idealTimeMs;
            this.actualTimeMs = actualTimeMs;
            this.dropped = dropped;
        }
    }

    static class Stats {
        int total = 0;
        int dropped = 0;
        double sumAbsJitter = 0.0;
        double maxAbsJitter = 0.0;
        double cpuBusyMs = 0.0;
    }

    public static void main(String[] args) {
        // Example suite chosen after a selection phase:
        // wheel_encoder + imu + 2d_lidar + gnss_rtk
        List<Sensor> sensors = Arrays.asList(
            new Sensor("wheel_encoder", 50.0, 0.3, 0.10, 0.001, 1),
            new Sensor("imu",          200.0, 0.4, 0.05, 0.0005, 0),
            new Sensor("2d_lidar",      10.0, 12.0, 1.20, 0.005, 2),
            new Sensor("gnss_rtk",       5.0, 1.5, 0.70, 0.002, 3)
        );

        double horizonMs = 60000.0; // 60 s
        Map<String, Stats> map = new LinkedHashMap<>();
        for (Sensor s : sensors) map.put(s.name, new Stats());

        PriorityQueue<Event> q = new PriorityQueue<>(
            Comparator.<Event>comparingDouble(e -> e.actualTimeMs)
                      .thenComparingInt(e -> e.sensor.priority)
        );

        Random rng = new Random(7);

        // Schedule all events
        for (Sensor s : sensors) {
            for (double t = 0.0; t <= horizonMs + 1e-9; t += s.periodMs()) {
                double jitter = rng.nextGaussian() * s.jitterStdMs;
                boolean dropped = rng.nextDouble() < s.dropProb;
                q.add(new Event(s, t, Math.max(0.0, t + jitter), dropped));
            }
        }

        double cpuAvailableAt = 0.0;
        while (!q.isEmpty()) {
            Event e = q.poll();
            Stats st = map.get(e.sensor.name);
            st.total += 1;

            if (e.dropped) {
                st.dropped += 1;
                continue;
            }

            double start = Math.max(e.actualTimeMs, cpuAvailableAt);
            double finish = start + e.sensor.processingMs;
            cpuAvailableAt = finish;

            double effectiveJitter = start - e.idealTimeMs; // includes queueing delay
            double absJ = Math.abs(effectiveJitter);

            st.sumAbsJitter += absJ;
            st.maxAbsJitter = Math.max(st.maxAbsJitter, absJ);
            st.cpuBusyMs += e.sensor.processingMs;
        }

        System.out.println("Sensor timing diagnostics (60 s horizon)");
        System.out.println("-----------------------------------------");
        double totalCpuBusy = 0.0;
        for (Sensor s : sensors) {
            Stats st = map.get(s.name);
            totalCpuBusy += st.cpuBusyMs;
            int valid = st.total - st.dropped;
            double meanAbsJitter = valid > 0 ? st.sumAbsJitter / valid : 0.0;
            double dropoutRate = st.total > 0 ? ((double) st.dropped / st.total) : 0.0;

            // A simple QoS score: penalize jitter, dropouts, and CPU time
            double qos = 100.0
                - 2.0 * meanAbsJitter
                - 0.8 * st.maxAbsJitter
                - 800.0 * dropoutRate
                - 0.005 * st.cpuBusyMs;

            System.out.printf(
                Locale.US,
                "%-12s total=%5d drop=%4d (%.3f) mean|j|=%.3f ms max|j|=%.3f ms cpu=%.1f ms QoS=%.2f%n",
                s.name, st.total, st.dropped, dropoutRate, meanAbsJitter, st.maxAbsJitter, st.cpuBusyMs, qos
            );
        }

        double cpuUtil = totalCpuBusy / horizonMs;
        System.out.printf(Locale.US, "%nAggregate CPU utilization = %.3f%n", cpuUtil);
        if (cpuUtil > 0.8) {
            System.out.println("Warning: CPU utilization exceeds recommended planning threshold (0.8).");
        } else {
            System.out.println("CPU utilization is within the planning threshold.");
        }
    }
}

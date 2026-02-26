/*
Chapter14_Lesson4.java
Recovery Behaviors and Fault Handling — ROS 2 Java (rcljava) oriented supervisor (teaching skeleton)

Notes:
- rcljava APIs evolve across ROS 2 distros. Treat this as a structured template.
- Core logic: progress window + oscillation score + recovery escalation.
*/

import java.util.ArrayDeque;
import java.util.Deque;

public class Chapter14_Lesson4 {

    static class DetParams {
        double windowS = 10.0;
        double minProgressM = 0.25;

        double oscWindowS = 6.0;
        int oscSignFlips = 6;
        double vEps = 0.03;

        int maxRecoveryAttempts = 3;

        double spinTimeS = 3.0;
        double spinWz = 0.8;
        double backupTimeS = 2.0;
        double backupVx = -0.12;
    }

    static class DistEntry {
        double t, d;
        DistEntry(double t, double d) { this.t = t; this.d = d; }
    }

    static class CmdEntry {
        double t, vx, wz;
        CmdEntry(double t, double vx, double wz) { this.t = t; this.vx = vx; this.wz = wz; }
    }

    static class SupervisorCore {
        DetParams p = new DetParams();

        Deque<DistEntry> progress = new ArrayDeque<>();
        Deque<CmdEntry> cmd = new ArrayDeque<>();

        int recoveryAttempts = 0;

        void addDistanceSample(double t, double distToGoal) {
            progress.addLast(new DistEntry(t, distToGoal));
            trim(progress, p.windowS);
        }

        void addCmdSample(double t, double vx, double wz) {
            cmd.addLast(new CmdEntry(t, vx, wz));
            trim(cmd, p.oscWindowS);
        }

        boolean progressOk() {
            if (progress.size() < 2) return true;
            double d0 = progress.peekFirst().d;
            double d1 = progress.peekLast().d;
            return (d0 - d1) >= p.minProgressM;
        }

        boolean oscillating() {
            if (cmd.size() < 3) return false;
            int flips = 0;

            CmdEntry prev = null;
            for (CmdEntry e : cmd) {
                if (prev != null) {
                    int sPrev = sgnEps(prev.vx);
                    int sNow  = sgnEps(e.vx);
                    if (sPrev != 0 && sNow != 0 && sPrev != sNow) flips++;

                    sPrev = sgnEps(prev.wz);
                    sNow  = sgnEps(e.wz);
                    if (sPrev != 0 && sNow != 0 && sPrev != sNow) flips++;
                }
                prev = e;
            }
            return flips >= p.oscSignFlips;
        }

        int sgnEps(double v) {
            if (Math.abs(v) < p.vEps) return 0;
            return (v > 0.0) ? 1 : -1;
        }

        void trim(Deque<?> q, double windowS) {
            if (q.isEmpty()) return;
            double tLatest;
            Object last = q.peekLast();
            if (last instanceof DistEntry) tLatest = ((DistEntry) last).t;
            else tLatest = ((CmdEntry) last).t;

            double tMin = tLatest - windowS;
            while (!q.isEmpty()) {
                Object first = q.peekFirst();
                double tFirst = (first instanceof DistEntry) ? ((DistEntry) first).t : ((CmdEntry) first).t;
                if (tFirst >= tMin) break;
                q.removeFirst();
            }
        }

        boolean shouldRecover() {
            return (!progressOk()) || oscillating();
        }

        boolean canContinue() {
            return recoveryAttempts < p.maxRecoveryAttempts;
        }

        void onRecoveryDone() {
            recoveryAttempts++;
        }
    }

    public static void main(String[] args) {
        // In a real robot, wrap SupervisorCore with ROS2 subscriptions/publishers:
        // - subscribe /odom (distance-to-goal computed from pose + goal)
        // - subscribe /cmd_vel (oscillation)
        // - call costmap clear services (Nav2) and publish temporary cmd_vel for spin/backup
        SupervisorCore core = new SupervisorCore();

        // Toy stream (stagnation + oscillation)
        core.addDistanceSample(0.0, 5.0);
        core.addDistanceSample(5.0, 4.95);
        core.addDistanceSample(10.0, 4.93);

        core.addCmdSample(0.0, 0.10, 0.0);
        core.addCmdSample(1.0, -0.10, 0.0);
        core.addCmdSample(2.0, 0.10, 0.0);
        core.addCmdSample(3.0, -0.10, 0.0);
        core.addCmdSample(4.0, 0.10, 0.0);
        core.addCmdSample(5.0, -0.10, 0.0);

        System.out.println("progressOk=" + core.progressOk());
        System.out.println("oscillating=" + core.oscillating());
        System.out.println("shouldRecover=" + core.shouldRecover());
    }
}

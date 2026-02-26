// Chapter18_Lesson5.java
// Case Study: Agricultural / Delivery Robots
// Mission-state machine and ETA/chance checks for outdoor AMR

import java.util.ArrayList;
import java.util.List;

public class Chapter18_Lesson5 {

    enum Mode {
        LOCALIZE,
        PLAN,
        EXECUTE,
        REPLAN,
        SAFE_STOP,
        COMPLETE
    }

    static class Segment {
        double meanTime;
        double stdTime;
        double risk;
        double energy;
        Segment(double meanTime, double stdTime, double risk, double energy) {
            this.meanTime = meanTime;
            this.stdTime = stdTime;
            this.risk = risk;
            this.energy = energy;
        }
    }

    static class Mission {
        String type;  // "agriculture" or "delivery"
        double deadlineSeconds;
        double batteryJoules;
        List<Segment> segments = new ArrayList<>();
        Mission(String type, double deadlineSeconds, double batteryJoules) {
            this.type = type;
            this.deadlineSeconds = deadlineSeconds;
            this.batteryJoules = batteryJoules;
        }
    }

    static double scoreMission(Mission m, double wt, double we, double wr) {
        double total = 0.0;
        for (Segment s : m.segments) {
            total += wt * s.meanTime + we * s.energy + wr * s.risk;
        }
        return total;
    }

    static boolean chanceDeadlineSatisfied(Mission m, double z) {
        double mu = 0.0;
        double var = 0.0;
        for (Segment s : m.segments) {
            mu += s.meanTime;
            var += s.stdTime * s.stdTime;
        }
        double sigma = Math.sqrt(var);
        return (mu + z * sigma) <= m.deadlineSeconds;
    }

    static boolean batterySatisfied(Mission m) {
        double energy = 0.0;
        for (Segment s : m.segments) {
            energy += s.energy;
        }
        return energy <= m.batteryJoules;
    }

    static Mode step(Mode mode, Mission mission) {
        switch (mode) {
            case LOCALIZE:
                return Mode.PLAN;
            case PLAN:
                if (!chanceDeadlineSatisfied(mission, 1.645)) return Mode.REPLAN;
                if (!batterySatisfied(mission)) return Mode.REPLAN;
                return Mode.EXECUTE;
            case EXECUTE:
                // Runtime check (placeholder for obstacle and confidence monitoring)
                boolean hazardDetected = false;
                boolean covarianceTooLarge = false;
                if (hazardDetected || covarianceTooLarge) return Mode.SAFE_STOP;
                return Mode.COMPLETE;
            case REPLAN:
                // In practice: lower speed, choose safer sidewalk/lane, or split mission
                return Mode.PLAN;
            case SAFE_STOP:
                return Mode.REPLAN;
            default:
                return Mode.COMPLETE;
        }
    }

    static Mission buildAgriculturalMission() {
        Mission m = new Mission("agriculture", 1800.0, 550000.0);
        m.segments.add(new Segment(250.0, 20.0, 0.22, 52000.0));
        m.segments.add(new Segment(310.0, 25.0, 0.35, 70000.0));
        m.segments.add(new Segment(280.0, 15.0, 0.18, 61000.0));
        return m;
    }

    static Mission buildDeliveryMission() {
        Mission m = new Mission("delivery", 420.0, 180000.0);
        m.segments.add(new Segment(110.0, 12.0, 0.12, 9000.0));
        m.segments.add(new Segment(95.0, 10.0, 0.20, 8000.0));
        m.segments.add(new Segment(150.0, 20.0, 0.28, 12000.0));
        return m;
    }

    public static void main(String[] args) {
        Mission ag = buildAgriculturalMission();
        Mission dr = buildDeliveryMission();

        double agScore = scoreMission(ag, 1.0, 0.0005, 12.0);
        double drScore = scoreMission(dr, 1.6, 0.0008, 8.0);

        System.out.println("Agricultural score = " + agScore);
        System.out.println("Agricultural deadline feasible = " + chanceDeadlineSatisfied(ag, 1.282)); // 90%
        System.out.println("Agricultural battery feasible = " + batterySatisfied(ag));

        System.out.println("Delivery score = " + drScore);
        System.out.println("Delivery deadline feasible = " + chanceDeadlineSatisfied(dr, 1.645)); // 95%
        System.out.println("Delivery battery feasible = " + batterySatisfied(dr));

        Mode mode = Mode.LOCALIZE;
        for (int k = 0; k < 10 && mode != Mode.COMPLETE; k++) {
            mode = step(mode, dr);
            System.out.println("State -> " + mode);
        }
    }
}

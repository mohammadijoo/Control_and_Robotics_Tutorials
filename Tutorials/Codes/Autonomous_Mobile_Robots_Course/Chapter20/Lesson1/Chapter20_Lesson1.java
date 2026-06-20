// Chapter20_Lesson1.java
// Chapter 20 - Lesson 1
// Problem Definition and Environment Setup (Java version)

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class Chapter20_Lesson1 {

    static class MissionSpec {
        String name;
        double startX, startY, startTheta;
        double goalX, goalY;
        double maxTimeS;
        double minSuccessProb;
        int maxCollisions;
        double mapResolutionM;
    }

    static class SensorSuite {
        double lidarRangeM, lidarFovDeg, lidarRateHz;
        double imuRateHz, wheelRateHz, gpsRateHz;
        boolean gpsAvailable;
    }

    static class RobotLimits {
        double radiusM, vMaxMps, wMaxRadps;
        double batteryWh, avgPowerW;
    }

    static class EnvironmentConfig {
        double widthM, heightM;
        double obstacleDensity;
        int slipRegions;
        long seed;
    }

    static class Grid2D {
        int rows, cols;
        int[][] occ;
        double[][] trav;
    }

    static Grid2D createGrid(EnvironmentConfig env, double resolutionM) {
        Grid2D g = new Grid2D();
        g.cols = (int) Math.ceil(env.widthM / resolutionM);
        g.rows = (int) Math.ceil(env.heightM / resolutionM);
        g.occ = new int[g.rows][g.cols];
        g.trav = new double[g.rows][g.cols];

        Random rand = new Random(env.seed);
        for (int r = 0; r < g.rows; r++) {
            for (int c = 0; c < g.cols; c++) {
                g.occ[r][c] = (rand.nextDouble() < env.obstacleDensity) ? 1 : 0;
                g.trav[r][c] = 1.0 + 0.5 * rand.nextDouble();
            }
        }

        int margin = Math.max(2, (int) Math.ceil(0.8 / resolutionM));
        for (int r = 0; r < g.rows; r++) {
            for (int c = 0; c < g.cols; c++) {
                if (r < margin || r >= g.rows - margin || c < margin || c >= g.cols - margin) {
                    g.occ[r][c] = 0;
                }
            }
        }

        for (int k = 0; k < env.slipRegions; k++) {
            int cx = rand.nextInt(g.cols);
            int cy = rand.nextInt(g.rows);
            int rad = Math.max(4, Math.min(g.rows, g.cols) / 10);
            for (int r = 0; r < g.rows; r++) {
                for (int c = 0; c < g.cols; c++) {
                    int dx = c - cx;
                    int dy = r - cy;
                    if (dx * dx + dy * dy <= rad * rad) {
                        g.trav[r][c] += 1.0 + 0.8 * rand.nextDouble();
                    }
                }
            }
        }

        for (int r = 0; r < g.rows; r++) {
            for (int c = 0; c < g.cols; c++) {
                if (g.occ[r][c] == 1) g.trav[r][c] = Double.POSITIVE_INFINITY;
            }
        }

        return g;
    }

    static double[] chooseStartGoal(Grid2D g, double resolutionM, long seed) {
        Random rand = new Random(seed + 20);
        int attempts = 2000;
        while (attempts-- > 0) {
            int sy = rand.nextInt(g.rows);
            int sx = rand.nextInt(g.cols);
            int gy = rand.nextInt(g.rows);
            int gx = rand.nextInt(g.cols);
            if (g.occ[sy][sx] == 1 || g.occ[gy][gx] == 1) continue;
            double dist = Math.hypot((gx - sx) * resolutionM, (gy - sy) * resolutionM);
            if (dist >= 0.4 * Math.max(g.rows, g.cols) * resolutionM) {
                return new double[]{sx * resolutionM, sy * resolutionM, gx * resolutionM, gy * resolutionM};
            }
        }
        return new double[]{0.0, 0.0, resolutionM, resolutionM};
    }

    static double batteryAvailableTimeS(RobotLimits robot) {
        return (robot.batteryWh / robot.avgPowerW) * 3600.0;
    }

    static void writeScenarioJson(String path, MissionSpec m, SensorSuite s, RobotLimits r, EnvironmentConfig e, Grid2D g) throws IOException {
        int occCount = 0;
        double travSum = 0.0;
        int travN = 0;
        for (int i = 0; i < g.rows; i++) {
            for (int j = 0; j < g.cols; j++) {
                if (g.occ[i][j] == 1) occCount++;
                if (Double.isFinite(g.trav[i][j])) {
                    travSum += g.trav[i][j];
                    travN++;
                }
            }
        }

        try (FileWriter fw = new FileWriter(path)) {
            fw.write("{\n");
            fw.write("  \"mission\": {\n");
            fw.write("    \"name\": \"" + m.name + "\",\n");
            fw.write("    \"start_xytheta\": [" + m.startX + ", " + m.startY + ", " + m.startTheta + "],\n");
            fw.write("    \"goal_xy\": [" + m.goalX + ", " + m.goalY + "],\n");
            fw.write("    \"max_time_s\": " + m.maxTimeS + ",\n");
            fw.write("    \"min_success_prob\": " + m.minSuccessProb + ",\n");
            fw.write("    \"max_collisions\": " + m.maxCollisions + ",\n");
            fw.write("    \"map_resolution_m\": " + m.mapResolutionM + "\n");
            fw.write("  },\n");
            fw.write("  \"occupancy_shape\": [" + g.rows + ", " + g.cols + "],\n");
            fw.write("  \"occupancy_occupied_fraction\": " + ((double) occCount / (g.rows * g.cols)) + ",\n");
            fw.write("  \"traversability_finite_mean\": " + (travN > 0 ? travSum / travN : 0.0) + "\n");
            fw.write("}\n");
        }
    }

    public static void main(String[] args) throws IOException {
        EnvironmentConfig env = new EnvironmentConfig();
        env.widthM = 60.0;
        env.heightM = 40.0;
        env.obstacleDensity = 0.14;
        env.slipRegions = 4;
        env.seed = 42;

        RobotLimits robot = new RobotLimits();
        robot.radiusM = 0.25;
        robot.vMaxMps = 1.2;
        robot.wMaxRadps = 1.8;
        robot.batteryWh = 180.0;
        robot.avgPowerW = 85.0;

        SensorSuite sensors = new SensorSuite();
        sensors.lidarRangeM = 15.0;
        sensors.lidarFovDeg = 270.0;
        sensors.lidarRateHz = 10.0;
        sensors.imuRateHz = 100.0;
        sensors.wheelRateHz = 50.0;
        sensors.gpsRateHz = 5.0;
        sensors.gpsAvailable = true;

        MissionSpec mission = new MissionSpec();
        mission.name = "Capstone_AMR_IndoorOutdoor_Mix";
        mission.maxTimeS = 1200.0;
        mission.minSuccessProb = 0.90;
        mission.maxCollisions = 0;
        mission.mapResolutionM = 0.2;

        Grid2D grid = createGrid(env, mission.mapResolutionM);
        double[] sg = chooseStartGoal(grid, mission.mapResolutionM, env.seed);
        mission.startX = sg[0];
        mission.startY = sg[1];
        mission.startTheta = 0.0;
        mission.goalX = sg[2];
        mission.goalY = sg[3];

        double tAvail = batteryAvailableTimeS(robot);
        boolean timeFeasible = tAvail >= mission.maxTimeS;
        boolean ratesOk = sensors.wheelRateHz >= 20.0 && sensors.imuRateHz >= 50.0 && sensors.lidarRateHz >= 5.0;

        System.out.println("Mission: " + mission.name);
        System.out.println("Start: (" + mission.startX + ", " + mission.startY + ", " + mission.startTheta + ")");
        System.out.println("Goal: (" + mission.goalX + ", " + mission.goalY + ")");
        System.out.println("Battery available [s]: " + tAvail);
        System.out.println("Time feasible? " + timeFeasible);
        System.out.println("Sensor-rate checks ok? " + ratesOk);

        writeScenarioJson("Chapter20_Lesson1_java_scenario.json", mission, sensors, robot, env, grid);
        System.out.println("Wrote Chapter20_Lesson1_java_scenario.json");
    }
}

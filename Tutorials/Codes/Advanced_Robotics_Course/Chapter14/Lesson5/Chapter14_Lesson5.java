public class TeamAssignment {

    public static int[] greedyAssign(double[][] C) {
        int nRobots = C.length;
        int nTasks = C[0].length;
        int[] taskOfRobot = new int[nRobots];
        boolean[] taskTaken = new boolean[nTasks];

        for (int i = 0; i < nRobots; ++i) {
            double best = Double.POSITIVE_INFINITY;
            int bestTask = -1;
            for (int j = 0; j < nTasks; ++j) {
                if (taskTaken[j]) {
                    continue;
                }
                if (C[i][j] < best) {
                    best = C[i][j];
                    bestTask = j;
                }
            }
            taskOfRobot[i] = bestTask;
            if (bestTask != -1) {
                taskTaken[bestTask] = true;
            }
        }
        return taskOfRobot;
    }

    public static void main(String[] args) {
        double[][] C = {
            {4.0, 2.0, 3.5},
            {2.5, 3.0, 2.0},
            {3.0, 4.0, 1.5}
        };
        int[] assignment = greedyAssign(C);
        for (int i = 0; i < assignment.length; ++i) {
            System.out.println("robot " + i + " -> task " + assignment[i]);
        }
    }
}
      

import java.util.*;

class Task {
    double C, T, D;
    Task(double C, double T, double D) { this.C = C; this.T = T; this.D = D; }
}

public class RTA {
    static boolean rtaSchedulable(List<Task> tasks) {
        tasks.sort(Comparator.comparingDouble(t -> t.T)); // RM order

        for (int i = 0; i < tasks.size(); i++) {
            Task ti = tasks.get(i);
            double R = ti.C;

            while (true) {
                double interference = 0.0;
                for (int j = 0; j < i; j++) {
                    Task tj = tasks.get(j);
                    interference += Math.ceil(R / tj.T) * tj.C;
                }
                double Rnext = ti.C + interference;
                if (Rnext == R) break;
                if (Rnext > ti.D) return false;
                R = Rnext;
            }
        }
        return true;
    }

    public static void main(String[] args) {
        List<Task> tasks = Arrays.asList(
            new Task(1.0, 5.0, 5.0),
            new Task(1.5, 8.0, 8.0),
            new Task(2.0, 12.0, 12.0)
        );
        System.out.println("Schedulable? " + rtaSchedulable(tasks));
    }
}

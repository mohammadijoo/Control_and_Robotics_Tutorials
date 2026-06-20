import java.util.ArrayList;
import java.util.List;
import java.util.Random;

class Agent {
    double x, y;
    double vx, vy;

    Agent(double x, double y) {
        this.x = x;
        this.y = y;
        this.vx = 0.0;
        this.vy = 0.0;
    }
}

public class SwarmSim {

    static List<Agent> initializeAgents(int N, double boxSize, long seed) {
        Random rng = new Random(seed);
        List<Agent> agents = new ArrayList<>(N);
        for (int i = 0; i < N; i++) {
            double x = boxSize * (rng.nextDouble() - 0.5);
            double y = boxSize * (rng.nextDouble() - 0.5);
            agents.add(new Agent(x, y));
        }
        return agents;
    }

    static void step(List<Agent> agents, double h, double R,
                     double wAtt, double wRep, double dMin, double vMax) {
        int N = agents.size();
        double R2 = R * R;

        // Reset velocities
        for (Agent a : agents) {
            a.vx = 0.0;
            a.vy = 0.0;
        }

        // Compute controls
        for (int i = 0; i < N; i++) {
            Agent ai = agents.get(i);
            double attX = 0.0, attY = 0.0;
            double repX = 0.0, repY = 0.0;

            for (int j = 0; j < N; j++) {
                if (j == i) continue;
                Agent aj = agents.get(j);
                double dx = aj.x - ai.x;
                double dy = aj.y - ai.y;
                double d2 = dx * dx + dy * dy;
                if (d2 <= R2) {
                    attX += dx;
                    attY += dy;
                    if (d2 > 1e-12) {
                        double d = Math.sqrt(d2);
                        if (d <= dMin) {
                            double scale = -1.0 / (d * d * d);
                            repX += dx * scale;
                            repY += dy * scale;
                        }
                    }
                }
            }
            ai.vx = wAtt * attX + wRep * repX;
            ai.vy = wAtt * attY + wRep * repY;
        }

        // Saturate and integrate
        for (Agent a : agents) {
            double speed2 = a.vx * a.vx + a.vy * a.vy;
            if (speed2 > vMax * vMax) {
                double speed = Math.sqrt(speed2);
                double scale = vMax / speed;
                a.vx *= scale;
                a.vy *= scale;
            }
            a.x += h * a.vx;
            a.y += h * a.vy;
        }
    }

    public static void main(String[] args) {
        int N = 50;
        double h = 0.05;
        double R = 2.0;
        double wAtt = 0.2;
        double wRep = 0.05;
        double dMin = 0.5;
        double vMax = 0.5;
        int steps = 500;

        List<Agent> agents = initializeAgents(N, 10.0, 0L);
        for (int k = 0; k < steps; k++) {
            step(agents, h, R, wAtt, wRep, dMin, vMax);
        }

        // Print final positions
        for (Agent a : agents) {
            System.out.println(a.x + " " + a.y);
        }
    }
}
      

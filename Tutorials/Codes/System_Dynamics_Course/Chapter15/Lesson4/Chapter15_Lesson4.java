// Chapter15_Lesson4.java
// Hybrid continuous-discrete simulation in Java
// Example: Thermostat with hysteresis (mode switching)

import java.util.ArrayList;
import java.util.List;

public class Chapter15_Lesson4 {

    enum Mode { HEATER_ON, HEATER_OFF }

    static class State {
        double t;
        double T;
        Mode mode;

        State(double t, double T, Mode mode) {
            this.t = t;
            this.T = T;
            this.mode = mode;
        }
    }

    // Parameters
    static final double T_ENV = 18.0;     // ambient temperature
    static final double TAU = 12.0;       // thermal time constant
    static final double Q_HEAT = 8.0;     // heating gain
    static final double T_LOW = 20.0;     // lower threshold
    static final double T_HIGH = 22.0;    // upper threshold

    static double f(double T, Mode mode) {
        double heater = (mode == Mode.HEATER_ON) ? Q_HEAT : 0.0;
        return -(T - T_ENV) / TAU + heater / TAU;
    }

    static double rk4Temperature(double T, Mode mode, double dt) {
        double k1 = f(T, mode);
        double k2 = f(T + 0.5 * dt * k1, mode);
        double k3 = f(T + 0.5 * dt * k2, mode);
        double k4 = f(T + dt * k3, mode);
        return T + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
    }

    static class EventResult {
        boolean switched;
        double tEvent;
        double TEvent;
        Mode nextMode;

        EventResult(boolean switched, double tEvent, double TEvent, Mode nextMode) {
            this.switched = switched;
            this.tEvent = tEvent;
            this.TEvent = TEvent;
            this.nextMode = nextMode;
        }
    }

    static EventResult detectAndRefineEvent(double t, double T, Mode mode, double dt) {
        double Tnext = rk4Temperature(T, mode, dt);

        if (mode == Mode.HEATER_ON && T < T_HIGH && Tnext >= T_HIGH) {
            // Bisection on phi(T) = T - T_HIGH
            double a = t, b = t + dt;
            double Ta = T, Tb = Tnext;
            for (int i = 0; i < 60; i++) {
                double m = 0.5 * (a + b);
                double Tm = rk4Temperature(Ta, mode, m - a);
                if (Math.abs(Tm - T_HIGH) < 1e-10 || (b - a) < 1e-10) {
                    return new EventResult(true, m, T_HIGH, Mode.HEATER_OFF);
                }
                if ((Ta - T_HIGH) * (Tm - T_HIGH) <= 0.0) {
                    b = m;
                    Tb = Tm;
                } else {
                    a = m;
                    Ta = Tm;
                }
            }
            return new EventResult(true, 0.5 * (a + b), T_HIGH, Mode.HEATER_OFF);
        }

        if (mode == Mode.HEATER_OFF && T > T_LOW && Tnext <= T_LOW) {
            // Bisection on phi(T) = T - T_LOW
            double a = t, b = t + dt;
            double Ta = T, Tb = Tnext;
            for (int i = 0; i < 60; i++) {
                double m = 0.5 * (a + b);
                double Tm = rk4Temperature(Ta, mode, m - a);
                if (Math.abs(Tm - T_LOW) < 1e-10 || (b - a) < 1e-10) {
                    return new EventResult(true, m, T_LOW, Mode.HEATER_ON);
                }
                if ((Ta - T_LOW) * (Tm - T_LOW) <= 0.0) {
                    b = m;
                    Tb = Tm;
                } else {
                    a = m;
                    Ta = Tm;
                }
            }
            return new EventResult(true, 0.5 * (a + b), T_LOW, Mode.HEATER_ON);
        }

        return new EventResult(false, t + dt, Tnext, mode);
    }

    public static void main(String[] args) {
        double tFinal = 48.0;
        double dt = 0.1;

        double t = 0.0;
        double T = 19.0;
        Mode mode = Mode.HEATER_ON;

        List<State> history = new ArrayList<>();
        history.add(new State(t, T, mode));

        while (t < tFinal) {
            EventResult er = detectAndRefineEvent(t, T, mode, dt);
            t = er.tEvent;
            T = er.TEvent;
            mode = er.nextMode;
            history.add(new State(t, T, mode));

            if (!er.switched) {
                continue;
            }
            // After a switch, continue integration with new mode.
        }

        System.out.println("t,Temperature,Mode");
        for (State s : history) {
            System.out.printf("%.6f,%.6f,%s%n", s.t, s.T, s.mode);
        }
    }
}

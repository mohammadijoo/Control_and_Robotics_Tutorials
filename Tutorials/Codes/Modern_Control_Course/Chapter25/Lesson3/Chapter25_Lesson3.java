/*
Chapter25_Lesson3.java

Trade-offs in SISO state-feedback for a second-order plant.

Compile:
    javac Chapter25_Lesson3.java

Run:
    java Chapter25_Lesson3
*/

public class Chapter25_Lesson3 {
    static class Vec2 {
        double x1;
        double x2;

        Vec2(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static class Mat2 {
        double a11;
        double a12;
        double a21;
        double a22;

        Mat2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class Metrics {
        double k1;
        double k2;
        double speed;
        double gainNorm;
        double effort;
        double maxAbsU;
    }

    static Vec2 add(Vec2 a, Vec2 b) {
        return new Vec2(a.x1 + b.x1, a.x2 + b.x2);
    }

    static Vec2 scale(Vec2 a, double c) {
        return new Vec2(c * a.x1, c * a.x2);
    }

    static Vec2 matVec(Mat2 A, Vec2 x) {
        return new Vec2(
            A.a11 * x.x1 + A.a12 * x.x2,
            A.a21 * x.x1 + A.a22 * x.x2
        );
    }

    static double control(double k1, double k2, Vec2 x) {
        return -(k1 * x.x1 + k2 * x.x2);
    }

    static Vec2 rk4Step(Mat2 Ac, Vec2 x, double dt) {
        Vec2 k1 = matVec(Ac, x);
        Vec2 k2 = matVec(Ac, add(x, scale(k1, 0.5 * dt)));
        Vec2 k3 = matVec(Ac, add(x, scale(k2, 0.5 * dt)));
        Vec2 k4 = matVec(Ac, add(x, scale(k3, dt)));

        Vec2 sum = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
        return add(x, scale(sum, dt / 6.0));
    }

    static Metrics simulateDesign(double desiredRealPart, double desiredImagPart) {
        double plantConst = 2.0;
        double plantSCoeff = 0.4;

        double a = desiredRealPart;
        double b = desiredImagPart;
        double desiredSCoeff = 2.0 * a;
        double desiredConst = a * a + b * b;

        double k1 = desiredConst - plantConst;
        double k2 = desiredSCoeff - plantSCoeff;

        Mat2 Ac = new Mat2(0.0, 1.0, -plantConst - k1, -plantSCoeff - k2);

        Vec2 x = new Vec2(1.0, 0.0);
        double dt = 0.0005;
        double tf = 10.0;
        int steps = (int)(tf / dt);

        double effort = 0.0;
        double maxAbsU = 0.0;

        for (int i = 0; i < steps; i++) {
            double u = control(k1, k2, x);
            effort += u * u * dt;
            maxAbsU = Math.max(maxAbsU, Math.abs(u));
            x = rk4Step(Ac, x, dt);
        }

        Metrics m = new Metrics();
        m.k1 = k1;
        m.k2 = k2;
        m.speed = a;
        m.gainNorm = Math.sqrt(k1 * k1 + k2 * k2);
        m.effort = effort;
        m.maxAbsU = maxAbsU;
        return m;
    }

    public static void main(String[] args) {
        String[] names = {"slow", "medium", "fast"};
        double[] realParts = {1.0, 3.0, 6.0};
        double[] imagParts = {1.0, 3.0, 6.0};

        System.out.println("State-feedback speed/effort trade-off");
        System.out.printf("%10s%12s%12s%12s%14s%16s%12s%n",
            "case", "speed", "k1", "k2", "||K||", "int u^2 dt", "max |u|");

        for (int i = 0; i < names.length; i++) {
            Metrics m = simulateDesign(realParts[i], imagParts[i]);
            System.out.printf("%10s%12.4f%12.4f%12.4f%14.4f%16.4f%12.4f%n",
                names[i], m.speed, m.k1, m.k2, m.gainNorm, m.effort, m.maxAbsU);
        }
    }
}

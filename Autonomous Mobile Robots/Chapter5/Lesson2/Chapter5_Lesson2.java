/*
Chapter 5 — Odometry and Dead Reckoning
Lesson 2: IMU Integration for Ground Robots

Filename: Chapter5_Lesson2.java

Compile/run:
  javac Chapter5_Lesson2.java
  java Chapter5_Lesson2

This example implements quaternion strapdown IMU integration (deterministic),
with optional planar constraints (yaw-only attitude, z=0).

No external libraries are required.
*/

import java.util.Locale;

public class Chapter5_Lesson2 {

  static class Vec3 {
    double x, y, z;
    Vec3(double x, double y, double z) { this.x=x; this.y=y; this.z=z; }
    Vec3 add(Vec3 o) { return new Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 sub(Vec3 o) { return new Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 mul(double s) { return new Vec3(x*s, y*s, z*s); }
    double norm() { return Math.sqrt(x*x + y*y + z*z); }
  }

  static class Mat3 {
    double[][] m = new double[3][3];
    Vec3 mul(Vec3 v) {
      return new Vec3(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
      );
    }
  }

  static class Quat { // w, x, y, z
    double w, x, y, z;
    Quat(double w, double x, double y, double z) { this.w=w; this.x=x; this.y=y; this.z=z; }

    void normalize() {
      double n = Math.sqrt(w*w + x*x + y*y + z*z);
      if (n <= 0) { w=1; x=y=z=0; return; }
      w/=n; x/=n; y/=n; z/=n;
    }

    static Quat mul(Quat a, Quat b) {
      return new Quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
      );
    }

    static Quat fromDeltaTheta(Vec3 dtheta) {
      double angle = dtheta.norm();
      if (angle < 1e-12) {
        return new Quat(1.0, 0.5*dtheta.x, 0.5*dtheta.y, 0.5*dtheta.z);
      }
      double ax = dtheta.x/angle, ay = dtheta.y/angle, az = dtheta.z/angle;
      double half = 0.5*angle;
      double s = Math.sin(half);
      return new Quat(Math.cos(half), ax*s, ay*s, az*s);
    }

    Mat3 toR() {
      Mat3 R = new Mat3();
      double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
      R.m[0][0] = 1 - 2*(yy + zz);
      R.m[0][1] = 2*(x*y - w*z);
      R.m[0][2] = 2*(x*z + w*y);

      R.m[1][0] = 2*(x*y + w*z);
      R.m[1][1] = 1 - 2*(xx + zz);
      R.m[1][2] = 2*(y*z - w*x);

      R.m[2][0] = 2*(x*z - w*y);
      R.m[2][1] = 2*(y*z + w*x);
      R.m[2][2] = 1 - 2*(xx + yy);
      return R;
    }

    double yaw() {
      return Math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
    }

    static Quat fromYaw(double yaw) {
      double half = 0.5*yaw;
      return new Quat(Math.cos(half), 0.0, 0.0, Math.sin(half));
    }
  }

  static double wrapPi(double a) {
    a = (a + Math.PI) % (2.0*Math.PI);
    if (a < 0) a += 2.0*Math.PI;
    return a - Math.PI;
  }

  public static void main(String[] args) {
    Locale.setDefault(Locale.US);

    // Synthetic demo: constant-speed turn
    int N = 2000;
    double dt = 0.01;
    double v0 = 1.2;
    double w0 = 0.20;
    double g = 9.80665;

    Vec3 p = new Vec3(0,0,0);
    Vec3 v = new Vec3(0,0,0);
    Quat q = new Quat(1,0,0,0);

    Vec3 gnav = new Vec3(0,0,-g);

    for (int k=0; k<N; ++k) {
      double t = k*dt;
      double yaw = w0*t;

      // True nav acceleration
      double ax = v0*w0*Math.cos(yaw);
      double ay = v0*w0*Math.sin(yaw);

      // specific force in body: f_b = R_bn (a - g)
      double c = Math.cos(yaw), s = Math.sin(yaw);
      double fx =  c*ax + s*ay;
      double fy = -s*ax + c*ay;
      double fz = g;

      Vec3 gyro = new Vec3(0,0,w0);
      Vec3 accel = new Vec3(fx, fy, fz);

      // Attitude propagation
      Quat dq = Quat.fromDeltaTheta(gyro.mul(dt));
      q = Quat.mul(q, dq);
      q.normalize();

      // Planar projection
      q = Quat.fromYaw(q.yaw());

      // a = R f + g
      Mat3 R = q.toR();
      Vec3 anav = R.mul(accel).add(gnav);

      // Integrate
      v = v.add(anav.mul(dt));
      p = p.add(v.mul(dt)).add(anav.mul(0.5*dt*dt));
      p.z = 0;
      v.z = 0;

      if (k % 100 == 0) {
        System.out.printf("t=%.2f p=(%.3f,%.3f) yaw[deg]=%.2f%n",
          t, p.x, p.y, Math.toDegrees(wrapPi(q.yaw())));
      }
    }
  }
}

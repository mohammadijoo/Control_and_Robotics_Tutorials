import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class StateSpaceVsTF {
  // Minimal complex class for demonstration
  static class C {
    double re, im;
    C(double re, double im){ this.re=re; this.im=im; }
    C add(C o){ return new C(re+o.re, im+o.im); }
    C sub(C o){ return new C(re-o.re, im-o.im); }
    C mul(C o){ return new C(re*o.re - im*o.im, re*o.im + im*o.re); }
    C div(C o){
      double den = o.re*o.re + o.im*o.im;
      return new C((re*o.re + im*o.im)/den, (im*o.re - re*o.im)/den);
    }
    double abs(){ return Math.hypot(re, im); }
    public String toString(){ return String.format("(% .6e%+.6ei)", re, im); }
  }

  public static void main(String[] args) {
    double m = 1.0, b = 0.4, k = 4.0;

    // A, B, C, D
    DMatrixRMaj A = new DMatrixRMaj(new double[][]{
      {0.0, 1.0},
      {-k/m, -b/m}
    });
    DMatrixRMaj B = new DMatrixRMaj(new double[][]{
      {0.0},
      {1.0/m}
    });
    DMatrixRMaj Cmat = new DMatrixRMaj(new double[][]{
      {1.0, 0.0}
    });
    double D = 0.0;

    double[] w = new double[]{0.01, 0.1, 1.0, 10.0};
    for (double wi : w) {
      // We evaluate G(jw) both ways.
      // State-space: G = C (sI - A)^{-1} B + D, with s = j w.

      // For a 2x2 system, do analytic inverse of (sI - A) with complex arithmetic:
      // sI - A = [[s, -1], [k, s + b]]
      C s = new C(0.0, wi);
      C a11 = s;              // s - 0
      C a12 = new C(-1.0, 0.0);
      C a21 = new C(k/m, 0.0); // -A(1,0) = k/m
      C a22 = new C(b/m, wi);  // s - (-b/m) = s + b/m = (b/m) + j w

      // determinant = a11*a22 - a12*a21
      C det = a11.mul(a22).sub(a12.mul(a21));

      // inverse = (1/det) * [[a22, -a12], [-a21, a11]]
      // Multiply by B = [0; 1/m], then by C = [1, 0]
      C inv11 = a22.div(det);
      C inv12 = (new C(1.0, 0.0)).div(det);      // -a12 = 1
      // Only need first row times B:
      // (inv11*0 + inv12*(1/m)) = inv12*(1/m)
      C Gss = inv12.mul(new C(1.0/m, 0.0)).add(new C(D, 0.0));

      // Transfer function polynomial: 1 / (m s^2 + b s + k)
      // s = j w, s^2 = -w^2
      C s2 = s.mul(s);
      C denom = (new C(m,0.0)).mul(s2).add((new C(b,0.0)).mul(s)).add(new C(k,0.0));
      C Gtf = (new C(1.0,0.0)).div(denom);

      System.out.println("w=" + wi + "  G_ss=" + Gss + "  G_tf=" + Gtf + "  diff=" + Gss.sub(Gtf).abs());
    }

    // Time simulation should be done via an ODE integrator (e.g., Apache Commons Math),
    // but the key lesson point is already shown: both frequency responses coincide.
    System.out.println("For time simulation in Java, use an ODE solver library and integrate xdot = A x + B u.");
  }
}

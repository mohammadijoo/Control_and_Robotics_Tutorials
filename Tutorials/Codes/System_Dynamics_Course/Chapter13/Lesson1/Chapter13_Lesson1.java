// Chapter13_Lesson1.java
// System Dynamics — Chapter 13, Lesson 1
// Modeling Two- and Multi-Degree-of-Freedom Mechanical Systems.
//
// This example uses Apache Commons Math for:
//   - Linear algebra (RealMatrix)
//   - ODE integration (DormandPrince853Integrator)
//
// Maven dependency (pom.xml):
//   <dependency>
//     <groupId>org.apache.commons</groupId>
//     <artifactId>commons-math3</artifactId>
//     <version>3.6.1</version>
//   </dependency>
//
// Run (example):
//   javac -cp commons-math3-3.6.1.jar Chapter13_Lesson1.java
//   java  -cp .:commons-math3-3.6.1.jar Chapter13_Lesson1
//
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.FixedStepHandler;
import org.apache.commons.math3.ode.sampling.StepNormalizer;

public class Chapter13_Lesson1 {

    static class TwoDOFParams {
        double m1 = 1.0, m2 = 0.8;
        double k1 = 200.0, k2 = 150.0, k3 = 100.0;
        double c1 = 1.5,   c2 = 1.0,   c3 = 0.8;
    }

    static RealMatrix diag(double a, double b) {
        double[][] d = {{a, 0.0},{0.0, b}};
        return MatrixUtils.createRealMatrix(d);
    }

    static RealMatrix twoDOF_C(TwoDOFParams p) {
        double[][] c = {
            {p.c1 + p.c2, -p.c2},
            {-p.c2, p.c2 + p.c3}
        };
        return MatrixUtils.createRealMatrix(c);
    }

    static RealMatrix twoDOF_K(TwoDOFParams p) {
        double[][] k = {
            {p.k1 + p.k2, -p.k2},
            {-p.k2, p.k2 + p.k3}
        };
        return MatrixUtils.createRealMatrix(k);
    }

    static class StateSpaceODE implements FirstOrderDifferentialEquations {
        private final RealMatrix A;
        private final RealMatrix B;

        StateSpaceODE(RealMatrix A, RealMatrix B) {
            this.A = A;
            this.B = B;
        }

        // Force input f(t) in R^2
        private RealVector force(double t) {
            // Free response: set to zero. Change here for forcing.
            return new ArrayRealVector(new double[]{0.0, 0.0});
        }

        @Override
        public int getDimension() {
            return A.getRowDimension();
        }

        @Override
        public void computeDerivatives(double t, double[] x, double[] xDot) {
            RealVector xv = new ArrayRealVector(x, false);
            RealVector fv = force(t);
            RealVector rhs = A.operate(xv).add(B.operate(fv));
            for (int i = 0; i < xDot.length; i++) {
                xDot[i] = rhs.getEntry(i);
            }
        }
    }

    public static void main(String[] args) {
        TwoDOFParams p = new TwoDOFParams();
        RealMatrix M = diag(p.m1, p.m2);
        RealMatrix C = twoDOF_C(p);
        RealMatrix K = twoDOF_K(p);

        // Build A, B for x = [q; qdot], xdot = A x + B f
        int n = 2;
        RealMatrix Minv = new LUDecomposition(M).getSolver().getInverse();

        RealMatrix Z = MatrixUtils.createRealMatrix(n, n);
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(n);

        RealMatrix A = MatrixUtils.createRealMatrix(2*n, 2*n);
        A.setSubMatrix(Z.getData(), 0, 0);
        A.setSubMatrix(I.getData(), 0, n);
        A.setSubMatrix(Minv.multiply(K).scalarMultiply(-1.0).getData(), n, 0);
        A.setSubMatrix(Minv.multiply(C).scalarMultiply(-1.0).getData(), n, n);

        RealMatrix B = MatrixUtils.createRealMatrix(2*n, n);
        B.setSubMatrix(Minv.getData(), n, 0);

        FirstOrderDifferentialEquations ode = new StateSpaceODE(A, B);

        double t0 = 0.0, tf = 8.0;
        double[] x0 = new double[]{0.02, -0.01, 0.0, 0.0};
        double[] xEnd = new double[x0.length];

        DormandPrince853Integrator integrator =
            new DormandPrince853Integrator(1e-4, 1e-2, 1e-10, 1e-10);

        // Print q1, q2 every 0.2 s
        integrator.addStepHandler(new StepNormalizer(0.2, new FixedStepHandler() {
            @Override
            public void handleStep(double t, double[] x, double[] xDot, boolean isLast) {
                System.out.printf(java.util.Locale.US, "t=%.2f  q1=% .6f  q2=% .6f%n", t, x[0], x[1]);
            }
        }));

        integrator.integrate(ode, t0, x0, tf, xEnd);
        System.out.printf(java.util.Locale.US, "Done. Final state: q1=% .6f  q2=% .6f%n", xEnd[0], xEnd[1]);
    }
}

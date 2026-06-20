// Chapter20_Lesson4.mo
// Chapter 20 — Chaos, Complex Dynamics, and Computational Tools
// Lesson 4 — Modelica model (Lorenz system)
//
// This is a minimal Modelica model. Simulate it in OpenModelica or Dymola.
//
// Suggested experiment: StopTime=40, Interval=0.01

model Chapter20_Lesson4_Lorenz
  parameter Real sigma = 10.0;
  parameter Real rho   = 28.0;
  parameter Real beta  = 8.0/3.0;

  Real x(start=1.0);
  Real y(start=1.0);
  Real z(start=1.0);

equation
  der(x) = sigma*(y - x);
  der(y) = x*(rho - z) - y;
  der(z) = x*y - beta*z;

annotation(
  experiment(StopTime=40, Interval=0.01)
);
end Chapter20_Lesson4_Lorenz;

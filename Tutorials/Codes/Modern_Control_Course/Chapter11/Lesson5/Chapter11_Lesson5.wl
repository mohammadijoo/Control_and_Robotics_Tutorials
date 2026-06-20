(* Chapter11_Lesson5.nb
   Wolfram Mathematica code for the finite-horizon LTV controllability Gramian. *)

ClearAll[A, B, t, w11, w12, w21, w22, W, t0, tf];

t0 = 0; tf = 4;
A[t_] := {{0, 1}, {-(2 + 0.60 Sin[t]), -0.25}};
B[t_] := {{0}, {1 + 0.45 Cos[t]}};
W[t_] := {{w11[t], w12[t]}, {w21[t], w22[t]}};

eqs = Thread[D[W[t], t] == A[t].W[t] + W[t].Transpose[A[t]] + B[t].Transpose[B[t]]];
ics = {w11[t0] == 0, w12[t0] == 0, w21[t0] == 0, w22[t0] == 0};

sol = NDSolveValue[Join[Flatten[eqs], ics], {w11, w12, w21, w22}, {t, t0, tf},
   WorkingPrecision -> 30, AccuracyGoal -> 15, PrecisionGoal -> 15];

Wc = N[{{sol[[1]][tf], sol[[2]][tf]}, {sol[[3]][tf], sol[[4]][tf]}}];
WcSym = (Wc + Transpose[Wc])/2;
Eigenvalues[WcSym]
Det[WcSym]
PositiveDefiniteMatrixQ[WcSym]

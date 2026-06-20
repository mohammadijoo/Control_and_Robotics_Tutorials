(* Chapter12_Lesson1.nb *)
(* Wolfram Language notebook-style code for controllability Gramians. *)

ClearAll["Global`*"];

A = {{0, 1}, {-2, -3}};
B = {{0}, {1}};
Tfinal = 2.0;

controllabilityMatrix[A_, B_] := Module[{n = Length[A]},
  ArrayFlatten[{Table[MatrixPower[A, k].B, {k, 0, n - 1}]}]
];

finiteHorizonGramian[A_, B_, T_] := Module[{n = Length[A], tau},
  NIntegrate[
    MatrixExp[A tau].B.Transpose[B].Transpose[MatrixExp[A tau]],
    {tau, 0, T},
    WorkingPrecision -> 30
  ]
];

Cmat = controllabilityMatrix[A, B];
MatrixRank[Cmat]

WcT = finiteHorizonGramian[A, B, Tfinal] // N;
WcT // MatrixForm
Eigenvalues[WcT] // N
MatrixRank[WcT]

(* Infinite-horizon Gramian by solving A.W + W.Transpose[A] + B.Transpose[B] == 0. *)
n = Length[A];
Wvars = Array[w, {n, n}];
eqns = Flatten[A.Wvars + Wvars.Transpose[A] + B.Transpose[B] == ConstantArray[0, {n, n}]];
sol = Solve[eqns, Flatten[Wvars]][[1]];
Winf = Wvars /. sol;
Winf // MatrixForm
Eigenvalues[Winf] // N

(* Uncontrollable example. *)
A2 = {{0, 0}, {0, -1}};
B2 = {{1}, {0}};
Cmat2 = controllabilityMatrix[A2, B2];
WcT2 = finiteHorizonGramian[A2, B2, Tfinal] // N;

MatrixRank[Cmat2]
WcT2 // MatrixForm
Eigenvalues[WcT2] // N
MatrixRank[WcT2]

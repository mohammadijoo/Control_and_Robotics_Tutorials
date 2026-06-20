(* ::Package:: *)

(* Chapter10_Lesson1.nb
   Mathematica / Wolfram Language code for finite-horizon state steering. *)

ClearAll["Global`*"];

T = 2.0;
Nsteps = 60;
dt = T/Nsteps;

Phi = {{1, dt}, {0, 1}};
Gamma = {{0.5 dt^2}, {dt}};

x0 = {{0}, {0}};
xf = {{1}, {0}};

S = Transpose[
   Table[
    Flatten[MatrixPower[Phi, Nsteps - 1 - k].Gamma],
    {k, 0, Nsteps - 1}
    ]
   ];

freeResponse = MatrixPower[Phi, Nsteps].x0;
targetShift = xf - freeResponse;

rankS = MatrixRank[S];

u = Transpose[S].Inverse[S.Transpose[S]].targetShift;

x = ConstantArray[0, {2, Nsteps + 1}];
x[[All, 1]] = Flatten[x0];

Do[
  x[[All, k + 1]] = Phi.x[[All, k]] + Flatten[Gamma] Flatten[u][[k]],
  {k, 1, Nsteps}
  ];

finalError = Norm[x[[All, -1]] - Flatten[xf]];
energy = dt Total[Flatten[u]^2];

Print["Rank of finite-horizon steering map S: ", rankS];
Print["Requested final state: ", Flatten[xf]];
Print["Achieved final state: ", x[[All, -1]]];
Print["Final steering error norm: ", ScientificForm[finalError]];
Print["Input energy approximation: ", energy];

ListLinePlot[
  {
   Transpose[{Subdivide[0, T, Nsteps], x[[1, All]]}],
   Transpose[{Subdivide[0, T, Nsteps], x[[2, All]]}]
   },
  PlotLegends -> {"position", "velocity"},
  AxesLabel -> {"time", "state"},
  PlotLabel -> "State steering: double integrator"
  ]

ListStepPlot[
  Transpose[{Subdivide[0, T - dt, Nsteps - 1], Flatten[u]}],
  AxesLabel -> {"time", "input u"},
  PlotLabel -> "Minimum-norm piecewise-constant steering input"
  ]

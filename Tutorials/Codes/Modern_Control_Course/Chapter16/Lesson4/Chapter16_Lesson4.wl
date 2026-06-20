(* Chapter16_Lesson4.nb *)

ClearAll[CCF, ControllabilityMatrixManual, PrintSystem];

CCF[denDesc_List, numDesc_List] := Module[
  {den = N[denDesc], num = N[numDesc], n, A, B, C, D, padded},
  If[Abs[den[[1]]] < 10^-14,
    Print["Leading denominator coefficient must be nonzero."]; Abort[]
  ];
  num = num/den[[1]];
  den = den/den[[1]];
  n = Length[den] - 1;
  If[Length[num] > n,
    Print["Only strictly proper systems are handled."]; Abort[]
  ];

  A = ConstantArray[0, {n, n}];
  Do[A[[i, i + 1]] = 1, {i, 1, n - 1}];
  A[[n, All]] = -Reverse[Rest[den]];

  B = ConstantArray[0, {n, 1}];
  B[[n, 1]] = 1;

  padded = Join[ConstantArray[0, n - Length[num]], num];
  C = {Reverse[padded]};
  D = {{0}};
  {A, B, C, D}
];

ControllabilityMatrixManual[A_, B_] := Module[
  {n = Length[A]},
  Transpose[Table[Flatten[MatrixPower[A, k].B], {k, 0, n - 1}]]
];

PrintSystem[name_String, denDesc_List, numDesc_List] := Module[
  {A, B, C, D, Wc, s, Gss, Gtf},
  {A, B, C, D} = CCF[denDesc, numDesc];
  Wc = ControllabilityMatrixManual[A, B];

  Print[""];
  Print[StringRepeat["=", 72]];
  Print[name];
  Print["A = ", MatrixForm[A]];
  Print["B = ", MatrixForm[B]];
  Print["C = ", MatrixForm[C]];
  Print["Wc = ", MatrixForm[Wc]];
  Print["rank(Wc) = ", MatrixRank[Wc]];
  Print["det(Wc) = ", Det[Wc]];

  Gss = Simplify[(C.Inverse[s IdentityMatrix[Length[A]] - A].B + D)[[1, 1]]];
  Gtf = Apart[FromDigits[Reverse[numDesc], s]/FromDigits[Reverse[denDesc], s]];
  Print["G(s) from state space = ", Gss];
  Print["G(s) from coefficients  = ", Gtf];
];

PrintSystem[
  "Example 1: G1(s) = 2/(s^2 + 3s + 2)",
  {1, 3, 2},
  {2}
];

PrintSystem[
  "Example 2: G2(s) = (s + 2)/(s^3 + 6s^2 + 11s + 6)",
  {1, 6, 11, 6},
  {1, 2}
];

PrintSystem[
  "Example 3: G3(s) = (0.5s^2 + 1.5s + 1)/(s^3 + 4s^2 + 5s + 2)",
  {1, 4, 5, 2},
  {0.5, 1.5, 1.0}
];

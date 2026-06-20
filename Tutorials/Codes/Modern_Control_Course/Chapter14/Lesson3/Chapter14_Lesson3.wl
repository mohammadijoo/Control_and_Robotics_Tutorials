(* Chapter14_Lesson3.nb / Wolfram Mathematica code cells *)
ClearAll[ctrb, obsv, rankReport];

ctrb[A_, B_] := ArrayFlatten[
  {Table[MatrixPower[A, k].B, {k, 0, Length[A] - 1}]}
];

obsv[A_, C_] := Join @@ Table[C.MatrixPower[A, k], {k, 0, Length[A] - 1}];

A = {{0, 1, 0}, {0, 0, 1}, {-2, -3, -4}};
B = {{0}, {0}, {1}};
Cmat = {{1, 0, 0}};

Ctrb = ctrb[A, B];
Obsv = obsv[A, Cmat];

CtrbDual = ctrb[Transpose[A], Transpose[Cmat]];
ObsvDual = obsv[Transpose[A], Transpose[B]];

rankReport = <|
  "rank Ctrb(A,B)" -> MatrixRank[Ctrb],
  "rank Obsv(A,C)" -> MatrixRank[Obsv],
  "rank Ctrb(A^T,C^T)" -> MatrixRank[CtrbDual],
  "rank Obsv(A^T,B^T)" -> MatrixRank[ObsvDual],
  "Obsv == Transpose[CtrbDual]" -> (Obsv == Transpose[CtrbDual]),
  "Transpose[Ctrb] == ObsvDual" -> (Transpose[Ctrb] == ObsvDual)
|>;

rankReport // MatrixForm

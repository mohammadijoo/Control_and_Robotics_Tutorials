(* tauMeas and tauPred are lists of equal length *)
validateDynamicsModel[tauMeas_List, tauPred_List, pParams_Integer] := Module[
  {
    e, n, rmse, y, yMean, eMean, varY, varE, vaf,
    num, den, fit, tss, rss, r2, sigma2Hat, aic, bic
  },
  n = Length[tauMeas];
  If[n != Length[tauPred],
    Return[$Failed, Module]
  ];
  e = tauMeas - tauPred;

  rmse = Sqrt[Mean[e^2]];

  y = tauMeas;
  yMean = Mean[y];
  eMean = Mean[e];

  varY = Variance[y, WorkingPrecision -> MachinePrecision];
  varE = Variance[e, WorkingPrecision -> MachinePrecision];

  vaf = 100.0 (1.0 - varE/varY);

  num = Norm[tauMeas - tauPred];
  den = Norm[tauMeas - ConstantArray[yMean, n]];
  fit = 100.0 (1.0 - num/den);

  tss = Total[(y - yMean)^2];
  rss = Total[e^2];
  r2 = 1.0 - rss/tss;

  sigma2Hat = Mean[e^2];
  aic = 2.0 pParams + n Log[sigma2Hat];
  bic = pParams Log[n] + n Log[sigma2Hat];

  <<Association[
    "RMSE" -> rmse,
    "VAF" -> vaf,
    "FitPercent" -> fit,
    "R2" -> r2,
    "AIC" -> aic,
    "BIC" -> bic
  ]
];

(* Example of residual autocorrelation up to lag L *)
residualAutocorrelation[e_List, L_Integer] := Module[{n, e0, r0, rho},
  n = Length[e];
  e0 = e - Mean[e];
  r0 = Total[e0^2]/n;
  rho = Table[
    If[l == 0, 1.0,
      Total[Take[e0, {l + 1, n}] * Take[e0, {1, n - l}]] / ((n - l) r0)
    ],
    {l, 0, L}
  ];
  rho
];
      

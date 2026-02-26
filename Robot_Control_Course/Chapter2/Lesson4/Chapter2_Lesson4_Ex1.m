
(* Parameters *)
J  = 0.5;
B  = 0.05;
Kp = Symbol["Kp"];
Kd = Symbol["Kd"];

(* Characteristic polynomial: J s^2 + (B + Kd) s + Kp = 0 *)
s  = Symbol["s"];
charPoly = J s^2 + (B + Kd) s + Kp;

(* Solve for poles in terms of Kp, Kd *)
poles = Solve[charPoly == 0, s]

(* Substitute design (zeta, wn) mapping *)
zeta = 0.7;
Ts   = 0.5;
wn   = 4/(zeta*Ts);

KpVal = J*wn^2;
KdVal = 2*J*zeta*wn - B;

polesNumeric = poles /. {Kp -> KpVal, Kd -> KdVal} // N

(* Step response of error dynamics for chosen gains *)
sys = TransferFunctionModel[KpVal/(J s^2 + (B + KdVal) s + KpVal), s];
stepResp = OutputResponse[sys, UnitStep[t], {t, 0, 2}];
Plot[stepResp, {t, 0, 2}, AxesLabel -> {"t", "e(t)"}]


% Power budget & runtime
Vb = 14.8; C_Ah = 5.0; eta = 0.9;
loads = [14.8 2.0; 5.0 1.2; 3.3 0.5];  % [V, Iavg]
P_tot = sum(loads(:,1).*loads(:,2));
E_b = 3600*Vb*C_Ah;
T_run_h = eta*E_b/P_tot/3600;

disp(['Average power [W] = ', num2str(P_tot)])
disp(['Estimated runtime [h] = ', num2str(T_run_h)])

% Wire voltage drop
rho = 1.68e-8; L = 1.5; A = 1.0e-6; I = 5.0;
Rw = rho*L/A;
dV = I*Rw;
disp(['Wire resistance [ohm] = ', num2str(Rw)])
disp(['Voltage drop [V] = ', num2str(dV)])

% RC cutoff
R = 1e3; C = 0.1e-6;
fc = 1/(2*pi*R*C);
disp(['Cutoff frequency [Hz] = ', num2str(fc)])

% Simulink note:
% Implement RC low-pass using a "Transfer Fcn" block with numerator [1]
% and denominator [R*C 1]. Feed sensor signal into it before ADC.
      
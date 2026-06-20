% Task requirements
tauL_peak = 35; omegaL_peak = 4; alphaL_peak = 12;
eta_g = 0.92;

% Catalog: [tau_cont tau_peak omega_max Jm mass cost eta]
catalog = [
    12  40  80 0.0008 1.2 300 0.90;
    20  60  40 0.0016 2.2 550 0.85;
    50 120  25 0.0030 6.0 900 0.70
];
names = ["BLDC-A","Servo-B","Hydro-C"];
gears = [5 8 12 16];

tau_samples = [0 10 30 10 0];
tau_rms_req = sqrt(mean(tau_samples.^2));

feasible = [];
for i=1:size(catalog,1)
    for g=gears
        tau_m_peak = (tauL_peak/(g*eta_g)) + g*catalog(i,4)*alphaL_peak;
        omega_m_peak = g*omegaL_peak;
        if tau_m_peak <= catalog(i,2) && omega_m_peak <= catalog(i,3) && tau_rms_req <= catalog(i,1)
            feasible = [feasible; i g tau_m_peak omega_m_peak];
        end
    end
end

disp("Feasible designs [motorIndex gear tau_m_peak omega_m_peak]:");
disp(feasible);

% Simulink note:
% Use a "Signal Builder" for tau_L(t), a Gain block for 1/(g*eta_g),
% and an Inertia block for g*Jm*alpha_L(t) to verify motor torque demand.
      

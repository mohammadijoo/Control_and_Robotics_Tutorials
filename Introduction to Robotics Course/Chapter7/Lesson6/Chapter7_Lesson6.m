% Candidate sensors: [rmin rmax fs Delta sigma_v cost]
S = [ ...
    0.0 5.0 200.0 0.01 0.02 50.0; 
    0.1 10.0 60.0 0.05 0.05 20.0; 
    0.0 8.0 120.0 0.02 0.03 35.0 ];

x_min_req = 0.0; x_max_req = 6.0;
B = 40.0; delta_x = 0.03; eps_max = 0.05;

feasible = true(size(S,1),1);
for i=1:size(S,1)
    rmin=S(i,1); rmax=S(i,2); fs=S(i,3);
    Delta=S(i,4); sigma_v=S(i,5);
    stot = sqrt(sigma_v^2 + Delta^2/12);

    feasible(i) = (rmin<=x_min_req) && (rmax>=x_max_req) ...
                  && (fs>=2*B) && (Delta<=delta_x) && (stot<=eps_max);
end

Sf = S(feasible,:);

if isempty(Sf)
    disp('No feasible sensors'); return;
end

sigma_tot = sqrt(Sf(:,5).^2 + Sf(:,4).^2/12);
latency = 1./Sf(:,3);
cost = Sf(:,6);

normalize = @(v) (v-min(v))./(max(v)-min(v)+1e-12);
A = [normalize(sigma_tot), normalize(latency), normalize(cost)];

w = [0.5 0.3 0.2]; % weights
scores = A*w';

[~,best_local] = min(scores);
best_global = find(feasible);
best_global = best_global(best_local);

disp('Best sensor index:'); disp(best_global);

% Simulink note:
% Create a subsystem "Candidate Sensor" with blocks:
% Gain for H, Band-Limited White Noise for v(t), and Zero-Order Hold for sampling.
% Use MATLAB Function block to compute feasible() and scores online.

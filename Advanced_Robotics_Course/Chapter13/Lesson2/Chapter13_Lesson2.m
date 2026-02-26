function [phi, simOut] = runEpisodeDR(seed)
    % seed: scalar for reproducibility
    rng(seed);

    % Nominal parameters
    m1_nom = 1.0; m2_nom = 1.0;
    mu_nom = 0.05;
    mass_rel_range = 0.3;
    mu_rel_range   = 0.5;

    % Sample randomized parameters
    m1 = m1_nom * (1 + mass_rel_range * (2*rand - 1));
    m2 = m2_nom * (1 + mass_rel_range * (2*rand - 1));
    mu = mu_nom * (1 + mu_rel_range   * (2*rand - 1));

    % Assign to base workspace (used by Simulink model "planar_arm_model.slx")
    assignin('base','m1',m1);
    assignin('base','m2',m2);
    assignin('base','mu_contact',mu);

    % Run simulation for one episode
    simOut = sim('planar_arm_model', 'StopTime', '5.0', ...
                 'SaveOutput', 'on', 'SaveState', 'off');

    % Collect return (assuming block outputs reward signal "r")
    r = simOut.r;  % timeseries
    dt = r.Time(2) - r.Time(1);
    gamma = 0.99;
    G = sum((gamma .^ (0:numel(r.Data)-1))' .* r.Data);

    % Output parameter vector and return for analysis
    phi = [m1; m2; mu];
    fprintf('Episode return: %f\n', G);
end
      

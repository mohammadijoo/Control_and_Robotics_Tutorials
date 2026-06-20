function demo_productization()
    % Failure rates for 3 series components [1/h]
    lambdas = [1e-4, 2e-4, 1.5e-4];
    t = 0:100:2000;  % mission time grid

    R_series = exp(-sum(lambdas) * t);

    % Cost comparison for designs A and B
    F_A = 5e4; F_B = 1.5e4;
    cA = 250; cB = 280;
    Y_A = 0.95; Y_B = 0.98;
    c_oA = 40;  c_oB = 35;

    N = 1:5000;
    c_unit_A = F_A ./ N + cA ./ Y_A + c_oA;
    c_unit_B = F_B ./ N + cB ./ Y_B + c_oB;

    figure;
    subplot(2,1,1);
    plot(t, R_series, 'LineWidth', 1.5);
    xlabel('Mission time t [h]');
    ylabel('R_{series}(t)');
    grid on;
    title('Series reliability vs. mission time');

    subplot(2,1,2);
    plot(N, c_unit_A, N, c_unit_B, 'LineWidth', 1.5);
    xlabel('Production volume N');
    ylabel('Unit cost');
    legend('Design A','Design B', 'Location', 'best');
    grid on;
    title('Unit cost vs. production volume');
end

% (Optional) Simulink setup for a simple derating calculation:
% model = 'derating_model';
% new_system(model);
% open_system(model);
% add_block('simulink/Sources/Constant', [model '/Load']);
% add_block('simulink/Math Operations/Gain', [model '/Derating']);
% set_param([model '/Derating'], 'Gain', '0.8');  % 20%% derating
% add_block('simulink/Sinks/Scope', [model '/Scope']);
% add_line(model, 'Load/1', 'Derating/1');
% add_line(model, 'Derating/1', 'Scope/1');
% save_system(model);
      

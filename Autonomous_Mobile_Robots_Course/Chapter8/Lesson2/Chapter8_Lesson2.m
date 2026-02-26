\
% Chapter 8 - Lesson 2: Importance Sampling and Resampling (Particle Filters)
%
% MATLAB implementation (from scratch) + optional programmatic Simulink model creation.
% Robotics-oriented MATLAB toolboxes where these ideas are used:
%   - Robotics System Toolbox (AMCL-related workflows, localization examples)
%   - Navigation Toolbox (state estimation / planners; PF concepts appear)
%
% Run in MATLAB:
%   Chapter8_Lesson2
%
% Note: This script is self-contained.

function Chapter8_Lesson2()
    rng(7);

    N = 2000;
    q = 0.4;  % process std
    r = 0.7;  % measurement std
    u = 0.3;  % commanded motion

    % initial particles
    x = 4.0 * randn(N,1);
    logw = zeros(N,1);

    x_true = 2.0;

    for t = 1:8
        x_true = x_true + u + q*randn();

        % propagate
        x = x + u + q*randn(N,1);

        % measurement
        z = x_true + r*randn();

        % importance weights: log-likelihood (ignore constants)
        logw = -0.5 * ((z - x)/r).^2;

        w = normalize_log_weights(logw);
        Neff = effective_sample_size(w);
        x_hat = sum(w .* x);

        fprintf('t=%02d, z=%+.3f, true=%+.3f, est=%+.3f, ESS=%.1f\n', ...
            t, z, x_true, x_hat, Neff);

        if Neff < 0.5*N
            a = systematic_resample(w);
            x = x(a);
            logw(:) = 0.0;
        end
    end

    % Optional: create a small Simulink skeleton with a MATLAB Function block.
    % Comment out if Simulink is not available.
    % create_simulink_skeleton();
end

function lse = log_sum_exp(logw)
    m = max(logw);
    lse = m + log(sum(exp(logw - m)));
end

function w = normalize_log_weights(logw)
    lse = log_sum_exp(logw);
    w = exp(logw - lse);
    w = w / sum(w);
end

function Neff = effective_sample_size(w)
    Neff = 1.0 / sum(w.^2);
end

function a = systematic_resample(w)
    N = length(w);
    cdf = cumsum(w);
    u0 = rand() / N;
    a = zeros(N,1);
    j = 1;
    for i = 1:N
        u = u0 + (i-1)/N;
        while u > cdf(j) && j < N
            j = j + 1;
        end
        a(i) = j;
    end
end

function create_simulink_skeleton()
    model = 'Chapter8_Lesson2_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    if exist([model '.slx'],'file')
        delete([model '.slx']);
    end

    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Constant', [model '/w_in'], 'Value', 'ones(10,1)/10');
    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/SystematicResample']);
    add_block('simulink/Sinks/To Workspace', [model '/a_out'], 'VariableName', 'a_out');

    set_param([model '/SystematicResample'], 'Script', matlabFunctionBlockCode());

    add_line(model, 'w_in/1', 'SystematicResample/1');
    add_line(model, 'SystematicResample/1', 'a_out/1');

    save_system(model);
end

function code = matlabFunctionBlockCode()
    % MATLAB Function block code as a char vector:
    code = [
        "function a = fcn(w)" newline ...
        "% Systematic resampling (educational; expects normalized weights)" newline ...
        "N = length(w);" newline ...
        "cdf = cumsum(w);" newline ...
        "u0 = rand()/N;" newline ...
        "a = zeros(N,1);" newline ...
        "j = 1;" newline ...
        "for i = 1:N" newline ...
        "    u = u0 + (i-1)/N;" newline ...
        "    while u > cdf(j) && j < N" newline ...
        "        j = j + 1;" newline ...
        "    end" newline ...
        "    a(i) = j;" newline ...
        "end" newline ...
    ];
end

function translate_specs_example()
    Mp_percent = 10;   % 10% overshoot
    Ts_max = 2.0;      % 2 s settling time

    zeta_min = zeta_from_Mp(Mp_percent);
    sigma_max = -4 / Ts_max;   % approx 2% criterion

    % Example plant: DC motor-like
    J = 0.01; b = 0.1; K = 0.5;
    num = K;
    den = [J b 0];
    G = tf(num, den);

    figure; rlocus(G);
    hold on;
    % Constant-zeta rays
    % "sgrid" can draw constant zeta and wn lines
    sgrid(zeta_min, []);
    % Vertical line for sigma_max
    x = [sigma_max sigma_max];
    y = ylim;
    plot(x, y, 'r--', 'LineWidth', 1.5);
    title('Root locus with time-domain constraint region');
    hold off;
end

function zeta = zeta_from_Mp(Mp_percent)
    Mp = Mp_percent / 100;
    lnMp = log(Mp);
    zeta = -lnMp / sqrt(pi^2 + lnMp^2);
end

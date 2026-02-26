% Define plant
s = tf('s');
G = 1 / (s * (s + 2));

% Root locus plot
figure;
rlocus(G);
title('Root locus of G(s) = 1 / (s (s + 2))');
grid on;

% Overlay a damping ratio line, e.g., zeta ≈ 0.6
zeta_target = 0.6;
wn_dummy = 1;  % sgrid draws rays independent of wn value
sgrid(zeta_target, wn_dummy);

% Option 1: Interactively pick a point on the locus to obtain K
% [K_star, poles_star] = rlocfind(G);

% Option 2: Programmatic search for gain that achieves desired zeta
K_vec = linspace(1.01, 20, 2000);
bestK = NaN;
for k = K_vec
    T = feedback(k * G, 1);
    p = pole(T);
    % dominant pole: largest real part
    [~, idx] = max(real(p));
    p_dom = p(idx);
    sigma = real(p_dom);
    wd    = imag(p_dom);
    wn    = sqrt(sigma^2 + wd^2);
    zeta  = -sigma / wn;
    if zeta >= zeta_target
        bestK = k;
        break;
    end
end

fprintf('Selected K* ≈ %.3f\n', bestK);

% Simulink: this K* can be used as the gain of a P-controller block

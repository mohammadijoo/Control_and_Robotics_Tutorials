d  = 2;
k  = 50;
Kp = 150;
Kd = 20;

m_vec = linspace(0.5, 2.5, 9);
zeta_vec = zeros(size(m_vec));
wn_vec   = zeros(size(m_vec));

for i = 1:length(m_vec)
    m = m_vec(i);
    wn   = sqrt((k + Kp) / m);
    zeta = (d + Kd) / (2 * sqrt(m * (k + Kp)));
    wn_vec(i)   = wn;
    zeta_vec(i) = zeta;

    G0 = tf(1, [m d k]);
    C  = tf([Kd Kp], 1);
    Tm = feedback(C * G0, 1);
    % step(Tm); hold on;  % visualize effect of m on step response
end

% Example: add high-frequency pole
omega_h = 30;
m_nom   = 1.0;
G0 = tf(1, [m_nom d k]);
Gh = G0 * tf(1, [1/omega_h 1]);  % true plant
C  = tf([Kd Kp], 1);
Th = feedback(C * Gh, 1);

figure;
step(Th);
title('Closed-loop step with unmodeled high-frequency pole');

% Bode and margins
Lh = C * Gh;
figure;
margin(Lh);
title('Loop-shape and margins with high-frequency pole');

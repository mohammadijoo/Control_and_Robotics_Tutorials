% Continuous-time plant: G(s) = K / (tau s + 1)
K = 5;
tau = 0.1;
s = tf('s');
G = K / (tau*s + 1);

Ts = 0.001;
Ttotal = 5;
t = 0:Ts:Ttotal-Ts;
omegas = [1 5 10 20];
U0 = 0.2;

Ghat = zeros(size(omegas));

for idx = 1:length(omegas)
    omega = omegas(idx);
    u = U0 * sin(omega * t);
    y = lsim(G, u, t);

    % Discard transient
    N = numel(t);
    u_ss = u(N/2:end);
    y_ss = y(N/2:end);

    % FFT-based estimation
    Nss = numel(u_ss);
    U = fft(u_ss);
    Y = fft(y_ss);
    w = 2*pi*(0:Nss-1)/(Nss*Ts);

    [~, kmin] = min(abs(w - omega));
    Ghat(idx) = Y(kmin) / U(kmin);

    fprintf('omega = %.2f rad/s, |Ghat| = %.3f, angle = %.3f rad\n', ...
            omega, abs(Ghat(idx)), angle(Ghat(idx)));
end

% Compare with theoretical frequency response
[mag, phase] = bode(G, omegas);
mag = squeeze(mag);
phase = squeeze(phase);

disp('Theoretical vs estimated magnitudes:');
disp([mag(:), abs(Ghat(:))]);

% ---------- Encoder: counts -> angle, velocity ----------
function [theta, omega] = encoder_angle_velocity(counts, N, Ts)
    theta = (2*pi/N) * counts(:);
    omega = [0; diff(theta)/Ts];
end

% Example
N = 1024; Ts = 1e-3;
counts = cumsum([0 1 1 0 -1 -1 0 1]);
[theta_hat, omega_hat] = encoder_angle_velocity(counts, N, Ts);

% ---------- IMU gyro integration (1D) ----------
function theta = integrate_gyro(omega_meas, Ts, theta0)
    omega_meas = omega_meas(:);
    theta = zeros(size(omega_meas));
    theta(1) = theta0;
    for k = 2:length(omega_meas)
        theta(k) = theta(k-1) + Ts * omega_meas(k-1);
    end
end

omega_meas = 0.3 + 0.01*randn(1000,1);
theta_from_gyro = integrate_gyro(omega_meas, Ts, 0);

% ---------- F/T reconstruction ----------
function w_hat = wrench_from_voltages(v, S_hat)
    % least squares: w_hat = argmin ||S_hat w - v||
    w_hat = S_hat \ v;
end

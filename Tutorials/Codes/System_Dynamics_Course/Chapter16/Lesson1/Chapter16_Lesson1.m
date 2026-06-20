\
% Chapter16_Lesson1.m
% Sampling, aliasing, and zero-order hold demonstration (MATLAB/Simulink companion)

clear; clc; close all;

fs = 80;                 % sampling frequency [Hz]
Ts = 1/fs;
f1 = 12;                 % in-band component
f2 = 55;                 % aliases because f2 > fs/2
A1 = 1.0; A2 = 0.7;
duration = 0.25;

fdense = 5000;
td = 0:1/fdense:duration;
x_cont = A1*sin(2*pi*f1*td) + A2*sin(2*pi*f2*td);

n = 0:floor(duration*fs)-1;
ts = n*Ts;
x_samp = A1*sin(2*pi*f1*ts) + A2*sin(2*pi*f2*ts);

% Zero-order hold on dense grid
k = floor(td/Ts) + 1;
k(k < 1) = 1;
k(k > numel(x_samp)) = numel(x_samp);
x_zoh = x_samp(k);

% Alias frequency utility
m = round(f2/fs);
f_alias = abs(f2 - m*fs);
if f_alias > fs/2
    f_alias = fs - f_alias;
end
fprintf('Alias of %.1f Hz at fs=%.1f Hz is %.1f Hz\n', f2, fs, f_alias);

% Time-domain plot
figure;
plot(td, x_cont, 'LineWidth', 1.2); hold on;
stairs(td, x_zoh, 'LineWidth', 1.1);
plot(ts, x_samp, 'o', 'MarkerSize', 5);
grid on;
xlabel('Time [s]');
ylabel('Amplitude');
title('Sampling and Zero-Order Hold');
legend('Continuous (dense reference)', 'ZOH output', 'Samples', 'Location', 'best');

% Spectrum plot (simple FFT view)
Nfft = 4096;
Xc = fft(x_cont, Nfft);
Xs = fft(x_samp, Nfft);
fc = (0:Nfft/2) * (fdense/Nfft);
fspec = (0:Nfft/2) * (fs/Nfft);

figure;
plot(fc, abs(Xc(1:Nfft/2+1)), 'LineWidth', 1.1); hold on;
plot(fspec, abs(Xs(1:Nfft/2+1)), 'LineWidth', 1.1);
xlim([0 120]);
grid on;
xlabel('Frequency [Hz]');
ylabel('Magnitude (unnormalized)');
title('Aliasing in the Frequency Domain');
legend('Dense-grid spectrum', 'Sampled-sequence spectrum', 'Location', 'best');

% Export CSV
T1 = table(td(:), x_cont(:), x_zoh(:), 'VariableNames', {'t_dense','x_cont','x_zoh'});
T2 = table(n(:), ts(:), x_samp(:), 'VariableNames', {'k','ts','x_sample'});
writetable(T1, 'Chapter16_Lesson1_matlab_dense.csv');
writetable(T2, 'Chapter16_Lesson1_matlab_samples.csv');

% Simulink note:
% You can reproduce the same experiment with blocks:
% Sine Wave (12 Hz) + Sine Wave (55 Hz) -> Sum -> Zero-Order Hold (Ts=1/fs)
% -> Scope / Spectrum Analyzer

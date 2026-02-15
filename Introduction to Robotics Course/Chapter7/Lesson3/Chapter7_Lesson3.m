% ----- Parameters -----
fs = 2e6;              % sampling rate
Tc = 2e-3;             % chirp duration
B  = 200e6;            % bandwidth
fc = 77e9;             % carrier (mmWave)
c  = 3e8;

k = B / Tc;            % chirp slope

% ----- Transmit chirp -----
t = (0:1/fs:Tc-1/fs).';
s = exp(1j*2*pi*(fc*t + 0.5*k*t.^2));

% ----- Simulate delayed echo + Doppler -----
d_true = 20;              % meters
vr_true = 3;              % m/s radial velocity
T_true = 2*d_true/c;

fd = 2*vr_true*fc/c;      % Doppler frequency

delaySamp = round(T_true*fs);
r = zeros(size(s));
r(delaySamp+1:end) = 0.8*s(1:end-delaySamp).*exp(1j*2*pi*fd*t(1:end-delaySamp));

% ----- Mix (dechirp) to get beat signal -----
bSig = r .* conj(s);

% ----- FFT to estimate beat frequency -----
N = length(bSig);
Bf = abs(fft(bSig));
[~, idx] = max(Bf(1:floor(N/2)));
fb_hat = (idx-1)*fs/N;

d_hat = (c/(2*k)) * fb_hat;

fprintf("Estimated range: %.2f m\n", d_hat);

% ----- Doppler via phase slope across chirps (toy: single chirp) -----
% For multi-chirp processing, stack beat signals and FFT over slow-time.

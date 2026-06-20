% MATLAB script: asymptotes and breakaway points
% Example open-loop transfer function for a robotic joint servo.
num = 1;                % no zeros
den = conv([1 2], conv([1 5], [1 20]));  % (s+2)(s+5)(s+20)

sys = tf(num, den);

% Root locus and automatic breakaway visualization
figure;
rlocus(sys); grid on;
title('Root locus with asymptotes for robotic joint servo');

% Asymptotes: poles and zeros
p = pole(sys);
z = zero(sys);

np = length(p);
nz = length(z);
qa = np - nz;
sigma_a = (sum(p) - sum(z)) / qa;
angles = (2*(0:qa-1) + 1) * pi / qa;

disp('Centroid sigma_a = ');
disp(sigma_a);
disp('Asymptote angles (deg) = ');
disp(angles * 180/pi);

% Breakaway candidates via symbolic toolbox
syms s K;
D = poly2sym(den, s);
N = poly2sym(num, s);

eqK = -D / N;
dKds = diff(eqK, s);
candidates = double(solve(dKds == 0, s));

disp('Real candidate breakaway points:');
for k = 1:length(candidates)
    if abs(imag(candidates(k))) < 1e-6
        fprintf('  s = %.4f\n', real(candidates(k)));
    end
end

% In Simulink, the same sys can be used in a feedback loop
% and the SISO Design Tool can display root locus interactively.

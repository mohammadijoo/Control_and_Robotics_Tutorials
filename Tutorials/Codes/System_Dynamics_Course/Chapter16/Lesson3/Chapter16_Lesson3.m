% Chapter16_Lesson3.m
% y[n] - 1.5 y[n-1] + 0.56 y[n-2] = 0.2 x[n] + 0.1 x[n-1]
b = [0.2 0.1];
a = [1 -1.5 0.56];

N = 20;
x = [1 zeros(1,N-1)];
h = filter(b, a, x);
disp('h[n] first 10 samples:'); disp(h(1:10).');

[H, w] = freqz(b, a, 5);
disp(table(w, abs(H), angle(H), 'VariableNames', {'w','Mag','Phase'}));

Ts = 1;
Gz = tf(b, a, Ts, 'Variable', 'z^-1');
disp(Gz);

p = pole(Gz); z = zero(Gz);
disp('Poles:'), disp(p)
disp('Zeros:'), disp(z)
disp(['Stable = ', num2str(all(abs(p) < 1))])

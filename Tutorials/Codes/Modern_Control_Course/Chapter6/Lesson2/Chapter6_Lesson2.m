% Example 5.1
A = [0 1; -2 -3];
B = [0; 1];
C = [1 1];
D = 0;

sys = ss(A,B,C,D);

% Internal candidate poles from A
pA = eig(A)

% Poles/zeros of the transfer function (cancellation handled in minimal TF form)
G = tf(sys);
p_tf = pole(G)
z_tf = zero(G)

% Transmission zeros (general definition; works for MIMO too)
z_t = tzero(sys)

% Visual check
pzmap(G); grid on;

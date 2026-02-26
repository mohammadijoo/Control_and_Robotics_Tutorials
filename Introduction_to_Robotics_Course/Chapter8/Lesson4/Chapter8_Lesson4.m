%% (A) Scalar weighted fusion
z = [1.02; 0.97; 1.10];
w = [4; 2; 1];
x_hat = (w'*z)/sum(w)

%% (B) Vector WLS fusion
H1 = eye(2);        z1 = [2.0; 1.0];
H2 = [1 0];         z2 = 1.8;
W1 = 5*eye(2);      W2 = 2;

A = H1'*W1*H1 + H2'*W2*H2;
b = H1'*W1*z1 + H2'*W2*z2;

x_hat_vec = A\b

%% (C) Discrete complementary filter
T = 0.01; omega_c = 2.0;
alpha = (omega_c*T)/(1+omega_c*T);

t = 0:T:2;
y1 = sin(t) + 0.05*randn(size(t));      % low-frequency sensor
u2 = gradient(y1,T) + 0.2*randn(size(t)); % high-frequency rate

xhat = zeros(size(t));
for k=2:length(t)
  xhat(k) = (1-alpha)*(xhat(k-1) + T*u2(k)) + alpha*y1(k);
end
plot(t,y1,t,xhat); legend('y1','fused');

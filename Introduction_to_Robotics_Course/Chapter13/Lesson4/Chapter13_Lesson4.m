Ts = 0.01; N = 2000; m_nom = 2.0;
t = (0:N-1)*Ts;
u = 2*sin(0.5*t);

% Load measured velocity v_meas from experiment/log
% Here we mock it:
v_meas = 0.5*sin(0.5*t) + 0.02*randn(size(t));

dv = diff(v_meas)/Ts;
y = dv - u(1:end-1)/m_nom;
Phi = -v_meas(1:end-1)/m_nom;

b_hat = (Phi*Phi')\(Phi*y');  % scalar LS
disp(["Estimated b = ", num2str(b_hat)]);

% Validation on a step input
Nv = 800; tv = (0:Nv-1)*Ts;
u_val = ones(1,Nv);  % step

v_real = zeros(1,Nv); v_sim = zeros(1,Nv);
b_true = 0.8; m_true = 2.0;

for k=1:Nv-1
    v_real(k+1) = v_real(k) + Ts*(u_val(k)-b_true*v_real(k))/m_true;
    v_sim(k+1)  = v_sim(k)  + Ts*(u_val(k)-b_hat *v_sim(k))/m_nom;
end

e = v_real - v_sim;
rmse = sqrt(mean(e.^2));
maxerr = max(abs(e));
disp(["RMSE = ", num2str(rmse), ", MaxErr = ", num2str(maxerr)]);
      

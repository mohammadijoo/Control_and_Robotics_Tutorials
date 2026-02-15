m = 2.0; g = 9.81; h = 1e-3; T = 1.0;
q = 0.3; v = -1.0;
k = 5000; d = 50;

contact_force = @(q,v) (q >= 0) * 0 + (q < 0) * (k*(-q) - d*v);

ts = 0:h:T;
qs = zeros(size(ts)); vs = zeros(size(ts));
for i=1:length(ts)
    f_ext = -m*g + contact_force(q,v);
    a = f_ext/m;

    v = v + h*a;  % symplectic Euler
    q = q + h*v;

    qs(i)=q; vs(i)=v;
end
disp([q v])
      

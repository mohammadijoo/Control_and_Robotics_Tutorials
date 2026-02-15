m = 1.0; b = 0.6; k = 4.0;
A = [0 1; -k/m -b/m];
B = [0; 1/m];

h = 0.2; T = 5.0;
N = floor(T/h);

x = zeros(2, N+1);
u = @(t) 1.0;

for i = 1:N
    x(:,i+1) = x(:,i) + h*(A*x(:,i) + B*u((i-1)*h));
end
disp(['final q = ', num2str(x(1,end))]);
      

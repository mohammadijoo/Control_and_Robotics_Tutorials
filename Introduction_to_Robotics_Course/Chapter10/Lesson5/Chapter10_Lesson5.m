% Parameters
Cbus = 3e-3;
Rbus = 0.08;
Voc  = 24;
Rint = 0.12;

Vmin  = 20;
Vshed = 21.2;

dt = 1e-4; T = 1.5;
t = 0:dt:T;
V = zeros(size(t)); V(1)=Voc;

shed = false;

Ilogic = 2.0;
Imotor_step = 8.0;
Iaux = 1.5;  % auxiliary compute rail

for k=1:length(t)-1
    Il = Ilogic + (t(k)>=0.5)*Imotor_step + (~shed)*Iaux;
    Is = max((Voc - V(k))/Rint, 0);

    dV = (Is - Il - V(k)/Rbus)/Cbus;
    V(k+1) = V(k) + dt*dV;

    if (V(k) < Vshed)
        shed = true;  % disconnect auxiliary rail
    end
end

plot(t,V); xlabel('time (s)'); ylabel('Vbus (V)');
yline(Vmin,'--'); yline(Vshed,':');
legend('Vbus','Vmin','Vshed');

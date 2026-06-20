
function interaction_simulation
  % Parameters
  Mr = 5.0;  Dr = 2.0;
  Ke = 500.0; De = 10.0;
  Md = 2.0;  Dd = 30.0; Kd = 200.0;
  x_d = 0.05;
  mode = "impedance";  % or "admittance"

  tspan = [0 5];
  x0 = [0; 0; 0; 0];   % [x; v; x_c; v_c]
  [t, x] = ode45(@(t, x) dyn(t, x, x_d, mode, Mr, Dr, Ke, De, Md, Dd, Kd), tspan, x0);

  figure; 
  subplot(2,1,1); plot(t, x(:,1)); xlabel('t'); ylabel('x');
  subplot(2,1,2); plot(t, x(:,3)); xlabel('t'); ylabel('x_c');
end

function dx = dyn(~, x, x_d, mode, Mr, Dr, Ke, De, Md, Dd, Kd)
  % x = [x; v; x_c; v_c]
  pos   = x(1);
  vel   = x(2);
  x_c   = x(3);
  v_c   = x(4);

  F_env = Ke * pos + De * vel;
  dx    = zeros(4,1);

  if mode == "impedance"
      e  = x_d - pos;
      ed = 0.0 - vel;
      F_u = Kd * e + Dd * ed;

      a  = (F_u + F_env - Dr * vel) / Mr;
      dx(1) = vel;
      dx(2) = a;
      dx(3) = 0.0;
      dx(4) = 0.0;

  else
      % Admittance
      a_c = (F_env - Dd * v_c - Kd * (x_c - x_d)) / Md;
      dx(3) = v_c;
      dx(4) = a_c;

      e_pos = x_c - pos;
      F_u = 1000.0 * e_pos;

      a  = (F_u + F_env - Dr * vel) / Mr;
      dx(1) = vel;
      dx(2) = a;
  end
end

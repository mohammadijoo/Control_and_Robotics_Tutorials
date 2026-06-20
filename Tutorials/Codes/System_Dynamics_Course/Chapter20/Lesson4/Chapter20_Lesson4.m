% Chapter20_Lesson4.m
%{
Chapter 20 — Chaos, Complex Dynamics, and Computational Tools
Lesson 4 — Computational Tools

MATLAB demonstration:
1) Lorenz ODE with ode45 (nonstiff) and ode15s (stiff-capable)
2) Event detection for Poincaré section (x=0 crossings, increasing direction)
3) Logistic map bifurcation tail data + Lyapunov exponent curve
4) Optional: Programmatic creation of a simple Simulink model for Lorenz

Outputs: CSV files written to a local folder "outputs_ch20_l4_matlab".
%}

function Chapter20_Lesson4()
  outDir = "outputs_ch20_l4_matlab";
  if ~exist(outDir, 'dir'), mkdir(outDir); end

  % Parameters
  p.sigma = 10.0;
  p.rho   = 28.0;
  p.beta  = 8.0/3.0;

  x0 = [1;1;1];
  tspan = [0, 40];

  % --- ODE45 with event detection ---
  opts = odeset('RelTol',1e-9,'AbsTol',1e-12,'MaxStep',0.02,'Events',@(t,x) ev_x0(t,x,p));
  [t, x, te, xe] = ode45(@(t,x) lorenz_rhs(t,x,p), tspan, x0, opts);

  writematrix([t, x], fullfile(outDir, "lorenz_traj.csv"));

  if ~isempty(xe)
    writematrix(xe, fullfile(outDir, "lorenz_poincare_x0.csv"));
  end

  % --- Compare with ode15s (useful when stiffness appears) ---
  % Lorenz is not stiff in this parameter set, but we show usage:
  opts15 = odeset('RelTol',1e-9,'AbsTol',1e-12,'MaxStep',0.02);
  [t15, x15] = ode15s(@(t,x) lorenz_rhs(t,x,p), tspan, x0, opts15); %#ok<ASGLU>
  % (Not exported to keep files minimal.)

  % --- Logistic map bifurcation data ---
  rVals = linspace(2.5, 4.0, 400);
  nTrans = 800; nKeep = 120;
  bif = zeros(numel(rVals)*nKeep, 2);
  idx = 1;
  for i = 1:numel(rVals)
    r = rVals(i);
    x = 0.2;
    for k = 1:nTrans, x = r*x*(1-x); end
    for k = 1:nKeep
      x = r*x*(1-x);
      bif(idx,:) = [r, x];
      idx = idx + 1;
    end
  end
  writematrix(bif, fullfile(outDir, "logistic_bifurcation.csv"));

  % --- Logistic map Lyapunov curve ---
  rVals2 = linspace(2.8, 4.0, 200);
  ly = zeros(numel(rVals2), 2);
  for i = 1:numel(rVals2)
    r = rVals2(i);
    ly(i,:) = [r, logistic_lyapunov(r, 0.2, 1000, 4000)];
  end
  writematrix(ly, fullfile(outDir, "logistic_lyapunov.csv"));

  % --- Optional: build a Simulink model programmatically ---
  % If you prefer manual building, create 3 Integrator blocks and one MATLAB Function
  % that outputs dx,dy,dz and wire them as xdot -> Integrator -> x.
  if exist('simulink', 'file')
    try
      build_simulink_lorenz(p);
    catch ME
      fprintf("Simulink model build failed: %s\n", ME.message);
    end
  end

  disp("Done. Outputs written to: " + outDir);
end

function dx = lorenz_rhs(~, x, p)
  X = x(1); Y = x(2); Z = x(3);
  dx = [ p.sigma*(Y - X);
         X*(p.rho - Z) - Y;
         X*Y - p.beta*Z ];
end

function [value, isterminal, direction] = ev_x0(~, x, ~)
  value = x(1);        % x == 0
  isterminal = 0;      % do not stop
  direction = +1;      % negative -> positive only
end

function lam = logistic_lyapunov(r, x0, nTrans, n)
  x = x0;
  for k = 1:nTrans, x = r*x*(1-x); end
  s = 0.0;
  for k = 1:n
    x = r*x*(1-x);
    d = abs(r*(1 - 2*x));
    if d < 1e-300, d = 1e-300; end
    s = s + log(d);
  end
  lam = s / n;
end

function build_simulink_lorenz(p)
  mdl = "Chapter20_Lesson4_Lorenz";
  if bdIsLoaded(mdl), close_system(mdl, 0); end
  new_system(mdl); open_system(mdl);

  % Add 3 integrators for x,y,z
  add_block('simulink/Continuous/Integrator', mdl + "/IntX", 'Position',[200 60 230 90]);
  add_block('simulink/Continuous/Integrator', mdl + "/IntY", 'Position',[200 130 230 160]);
  add_block('simulink/Continuous/Integrator', mdl + "/IntZ", 'Position',[200 200 230 230]);

  % Set initial conditions
  set_param(mdl + "/IntX", 'InitialCondition', '1');
  set_param(mdl + "/IntY", 'InitialCondition', '1');
  set_param(mdl + "/IntZ", 'InitialCondition', '1');

  % MATLAB Function block for derivatives
  add_block('simulink/User-Defined Functions/MATLAB Function', mdl + "/LorenzF", 'Position',[60 110 140 180]);
  code = sprintf([ ...
    "function dx = f(x)\n" ...
    "%% x = [X;Y;Z]\n" ...
    "sigma = %.17g; rho = %.17g; beta = %.17g;\n" ...
    "X = x(1); Y = x(2); Z = x(3);\n" ...
    "dx = [ sigma*(Y-X); X*(rho-Z)-Y; X*Y - beta*Z ];\n" ...
    "end\n"], p.sigma, p.rho, p.beta);
  set_param(mdl + "/LorenzF", 'Script', code);

  % Mux and Demux
  add_block('simulink/Signal Routing/Mux', mdl + "/Mux", 'Inputs','3','Position',[160 110 180 180]);
  add_block('simulink/Signal Routing/Demux', mdl + "/Demux", 'Outputs','3','Position',[150 60 170 230]);

  % Connect integrator outputs -> mux -> function -> demux -> integrator inputs
  add_line(mdl, "IntX/1", "Mux/1");
  add_line(mdl, "IntY/1", "Mux/2");
  add_line(mdl, "IntZ/1", "Mux/3");
  add_line(mdl, "Mux/1", "LorenzF/1");
  add_line(mdl, "LorenzF/1", "Demux/1");
  add_line(mdl, "Demux/1", "IntX/1");
  add_line(mdl, "Demux/2", "IntY/1");
  add_line(mdl, "Demux/3", "IntZ/1");

  % Add scopes
  add_block('simulink/Sinks/Scope', mdl + "/Scope", 'Position',[270 120 310 160]);
  add_line(mdl, "Mux/1", "Scope/1");

  set_param(mdl, 'StopTime', '40');
  save_system(mdl);
end

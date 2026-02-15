
function u_safe = cbf_filter_1d(x, u_nom, gamma)
% CBF filter for 1D system x_dot = u
% Enforces x >= 0 with barrier h(x) = x and alpha(s) = gamma * s

u_min = -gamma * x;   % from constraint u + gamma * x >= 0
u_safe = max(u_nom, u_min);
end

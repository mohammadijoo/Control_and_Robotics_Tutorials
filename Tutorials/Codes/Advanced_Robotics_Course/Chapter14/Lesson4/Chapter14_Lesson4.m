function agents = step_orca_matlab(agents, T_h, delta)
% agents: struct array with fields p (2x1), v (2x1), v_pref (2x1), r, max_speed

N = numel(agents);
new_v = zeros(2, N);

for i = 1:N
    cons = orca_constraints_for_agent(i, agents, T_h, delta); % cell array of structs
    v_star = project_velocity(agents(i).v_pref, cons, agents(i).max_speed);
    new_v(:, i) = v_star;
end

for i = 1:N
    agents(i).v = new_v(:, i);
    agents(i).p = agents(i).p + delta * agents(i).v;
end
end

function cons = orca_constraints_for_agent(i, agents, T_h, delta)
ai = agents(i);
cons = {};
for j = 1:numel(agents)
    if j == i, continue; end
    aj = agents(j);
    p_ij = aj.p - ai.p;
    v_ij = ai.v - aj.v;
    R = ai.r + aj.r;
    ttc = time_to_collision(p_ij, v_ij, R, T_h);
    if ~isfinite(ttc), continue; end

    p_coll = p_ij + ttc * v_ij;
    n_ij = normalize_vec(p_coll);
    u_ij = (R * n_ij - p_coll) / max(ttc, delta);
    hp.n = n_ij;
    hp.p0 = ai.v + 0.5 * u_ij;
    cons{end+1} = hp; %#ok<AGROW>
end
end

function ttc = time_to_collision(p_ij, v_ij, R, T_h)
a = dot(v_ij, v_ij);
c = dot(p_ij, p_ij) - R^2;
if c <= 0
    ttc = 0;
    return;
end
if a == 0
    ttc = inf;
    return;
end
b = 2 * dot(p_ij, v_ij);
disc = b^2 - 4*a*c;
if disc < 0
    ttc = inf;
    return;
end
t_min = (-b - sqrt(disc)) / (2*a);
if t_min < 0 || t_min > T_h
    ttc = inf;
else
    ttc = t_min;
end
end

function v = project_velocity(v_pref, cons, max_speed)
v = v_pref(:);
nrm = norm(v);
if nrm > max_speed
    v = (max_speed / nrm) * v;
end
for k = 1:numel(cons)
    hp = cons{k};
    if (v - hp.p0).' * hp.n < 0
        alpha = (hp.p0 - v).' * hp.n;
        v = v + alpha * hp.n;
        nrm = norm(v);
        if nrm > max_speed
            v = (max_speed / nrm) * v;
        end
    end
end
end

function v = normalize_vec(v)
n = norm(v);
if n > 1e-9
    v = v / n;
end
end
      

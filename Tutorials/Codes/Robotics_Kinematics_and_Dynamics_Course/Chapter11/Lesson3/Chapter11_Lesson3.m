function H = total_energy(q, dq)
    T = kinetic_energy(q, dq);
    U = potential_energy(q);
    H = T + U;
end

function T = kinetic_energy(q, dq)
    Mq = M_matrix(q);
    T = 0.5 * dq.' * Mq * dq;
end

function sigma = scalar_sigma(q, dq)
    Mdot = directional_M_dot(q, dq);
    Cq = C_matrix(q, dq);
    middle = 0.5 * Mdot - Cq;
    sigma = dq.' * middle * dq;
end

function Mdot = directional_M_dot(q, dq)
    eps = 1e-6;
    q_plus = q + eps * dq;
    q_minus = q - eps * dq;
    M_plus = M_matrix(q_plus);
    M_minus = M_matrix(q_minus);
    Mdot = (M_plus - M_minus) / (2 * eps);
end

% User must implement:
%   M_matrix(q)  - inertia matrix
%   C_matrix(q,dq) - Coriolis matrix
%   g_vector(q) - gravity vector
%   potential_energy(q) - potential
      

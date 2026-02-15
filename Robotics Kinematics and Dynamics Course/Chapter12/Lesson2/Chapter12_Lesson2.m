function [f_list, n_list, tau] = ne_backward(F_list, N_list, R_ip1_i_list, ...
                                             p_i_list, z_list, joint_type_list, ...
                                             f_tip, n_tip)
% F_list, N_list: 3xN
% R_ip1_i_list: 3x3xN
% p_i_list, z_list: 3xN
% joint_type_list: 1xN char array: 'R' or 'P'
N = size(F_list, 2);
f_list = zeros(3, N);
n_list = zeros(3, N);
tau = zeros(1, N);

f_next = f_tip(:);
n_next = n_tip(:);

for i = N:-1:1
    R_ip1_i = R_ip1_i_list(:, :, i);
    p_i = p_i_list(:, i);
    F_i = F_list(:, i);
    N_i = N_list(:, i);
    z_i = z_list(:, i);
    jt = joint_type_list(i);

    f_child_i = R_ip1_i * f_next;
    n_child_i = R_ip1_i * n_next + cross(p_i, f_child_i);

    f_i = F_i + f_child_i;
    n_i = N_i + n_child_i;

    f_list(:, i) = f_i;
    n_list(:, i) = n_i;

    if jt == 'R'
        tau(i) = z_i.' * n_i;
    else
        tau(i) = z_i.' * f_i;
    end

    f_next = f_i;
    n_next = n_i;
end
end
      

function epsilon = check_cycle_consistency(T_WB, T_BC, T_WC_meas)
%CHECK_CYCLE_CONSISTENCY  Frobenius inconsistency over a 3-node cycle.

T_CW_meas = inv(T_WC_meas);
E = T_WB * T_BC * T_CW_meas;
epsilon = norm(E - eye(4), 'fro');
end

% Example usage:
T_WB = eye(4); T_WB(1,4) = 1;
T_BC = eye(4); T_BC(2,4) = 2;
T_WC_derived = T_WB * T_BC;
T_WC_meas = T_WC_derived; T_WC_meas(1,4) = T_WC_meas(1,4) + 0.05;

eps_val = check_cycle_consistency(T_WB, T_BC, T_WC_meas);
disp(eps_val);

% Plant: simple second-order joint model
J = 0.01; B = 0.1; K = 0;
G = tf(1, [J B K+1]);   % G(s) = 1 / (J s^2 + B s + 1)

Kp = 50; Ki = 0; Kd = 2; N = 20;  % filtered derivative parameter N
C = pid(Kp, Ki, Kd, 1/N);        % Mathematically: Kd * (N s)/(1 + N s)
                                 % Some MATLAB versions parameterize the filter as Td/N

T = feedback(C*G, 1);            % closed-loop transfer from reference to output

t = linspace(0, 2, 1000);
[y, t] = step(T, t);

figure;
plot(t, y, "LineWidth", 1.5); grid on;
xlabel("time (s)");
ylabel("joint position");
title("Step response with filtered derivative PID");

% In Simulink, one can use a PID Controller block and specify the
% filter coefficient N directly in the block parameters.

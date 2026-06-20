% L0: driver stub
function [counts, cpr] = read_encoder()
    persistent c; if isempty(c), c=0; end
    cpr = 4096;
    counts = c;
end

function write_pwm(u)
    persistent c; if isempty(c), c=0; end
    c = c + int32(0.1*u);
end

% L2: controller
function u = p_controller(ref, pos, kp)
    u = kp*(ref - pos);
end

% main (L3 app + L1 loop)
kp = 0.2; dt = 0.01; t = 0;
for k=1:400
    if t > 1.0, ref = 10; else, ref = 0; end

    [counts, cpr] = read_encoder();
    pos = double(counts)/double(cpr)*2*pi;

    u = p_controller(ref, pos, kp);
    write_pwm(u);

    t = t + dt;
end
      

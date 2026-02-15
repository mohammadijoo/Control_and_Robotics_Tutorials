% Parameters
portName   = "/dev/ttyUSB0";
baudRate   = 115200;
N          = 2000;
alpha      = 0.9;  % exponential filter coefficient

% Open serial port
s = serialport(portName, baudRate);
configureTerminator(s, "LF");

raw = zeros(N,1);
y   = zeros(N,1);

for k = 1:N
    line = readline(s);
    value = str2double(line);   % assume line is just a number
    if isnan(value)
        value = raw(max(k-1,1)); % simple hold on NaN
    end
    raw(k) = value;
    if k == 1
        y(k) = value;
    else
        y(k) = alpha * y(k-1) + (1-alpha) * value;
    end
end

clear s;  % close port

mean_raw = mean(raw);
std_raw  = std(raw, 1);
mean_y   = mean(y);
std_y    = std(y, 1);

fprintf("Raw mean = %.4f, raw std = %.4f\n", mean_raw, std_raw);
fprintf("Filtered mean = %.4f, filtered std = %.4f\n", mean_y, std_y);
      

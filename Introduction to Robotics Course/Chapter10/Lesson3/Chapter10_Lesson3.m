% UART in Matlab
s = serialport("/dev/ttyUSB0", 115200, "Timeout", 0.01);

v = single(0.8);
pkt = [hex2dec("AA"); typecast(v, "uint8").'];
write(s, pkt, "uint8");

rx = read(s, 5, "uint8");
if numel(rx) == 5 && rx(1) == hex2dec("55")
    echo = typecast(uint8(rx(2:5)), "single");
    disp(["Echo: ", num2str(echo)]);
end

% CAN (requires Vehicle Network Toolbox)
canCh = canChannel("SocketCAN", "can0", 500000);
start(canCh);
msgTx = canMessage(288, false, [1 2 3 4]); % 0x120 = 288
transmit(canCh, msgTx);

msgRx = receive(canCh, 1, "Timeout", 0.1);
if ~isempty(msgRx)
    disp(msgRx.ID);
    disp(msgRx.Data);
end
stop(canCh);

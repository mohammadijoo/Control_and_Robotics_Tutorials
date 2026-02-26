import serial
import struct
import time
import numpy as np

PORT = "/dev/ttyUSB0"
BAUD = 115200

# Simple packet: [0xAA][cmd][len][payload...][crc16]
def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def make_pkt(cmd, payload=b""):
    header = bytes([0xAA, cmd, len(payload)])
    body = header + payload
    crc = crc16_ccitt(body)
    return body + struct.pack(">H", crc)

def read_pkt(ser):
    # naive sync on 0xAA
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0xAA:
            break
    cmd = ser.read(1)[0]
    ln  = ser.read(1)[0]
    payload = ser.read(ln)
    crc_rx = struct.unpack(">H", ser.read(2))[0]
    crc_ok = (crc_rx == crc16_ccitt(bytes([0xAA, cmd, ln]) + payload))
    return cmd, payload, crc_ok

with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
    # 1) ping
    ser.write(make_pkt(0x01))
    resp = read_pkt(ser)
    assert resp and resp[2], "Ping CRC failed"
    print("Ping OK")

    # 2) read 200 sensor samples
    samples = []
    for _ in range(200):
        ser.write(make_pkt(0x10))
        cmd, payload, ok = read_pkt(ser)
        assert ok and cmd == 0x10
        x = struct.unpack(">f", payload)[0]
        samples.append(x)
        time.sleep(0.01)

    samples = np.array(samples)
    print("mean =", samples.mean(), "std =", samples.std())

    # z-score fault check with expected bounds
    mu0, sigma0 = 0.0, 0.02
    z = (samples.mean() - mu0) / (sigma0 / np.sqrt(len(samples)))
    print("z =", z)
    if abs(z) > 3.0:
        print("WARNING: sensor bias/drift suspected")

# UART example (pyserial)
import serial, struct, time

ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0.01)

def send_velocity(v):
    # pack float into 4 bytes little endian
    payload = struct.pack("<f", float(v))
    ser.write(b'\xAA' + payload)  # 0xAA as a simple header

def read_reply():
    if ser.in_waiting >= 5:
        hdr = ser.read(1)
        data = ser.read(4)
        if hdr == b'\x55':
            return struct.unpack("<f", data)[0]
    return None

send_velocity(0.8)
time.sleep(0.01)
print("Motor echoed:", read_reply())

# I2C example (smbus2)
from smbus2 import SMBus

IMU_ADDR = 0x68
WHO_AM_I = 0x75
with SMBus(1) as bus:
    who = bus.read_byte_data(IMU_ADDR, WHO_AM_I)
    print("IMU WHO_AM_I =", hex(who))

# CAN example (python-can)
import can

bus = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=500000)
msg = can.Message(arbitration_id=0x120, data=[0x01, 0x02, 0x03, 0x04], is_extended_id=False)
bus.send(msg)

rx = bus.recv(timeout=0.1)
if rx is not None:
    print("RX id:", hex(rx.arbitration_id), "data:", list(rx.data))

import serial
import numpy as np

# Configuration (adapt to your board)
PORT = "/dev/ttyUSB0"
BAUD = 115200
N_SAMPLES = 1000

# Calibration parameters for this axis (from offline least squares)
a_hat = 0.0123  # units per ADC code
b_hat = -0.15   # units

def open_serial(port, baud):
    return serial.Serial(port=port, baudrate=baud, timeout=1.0)

def parse_line(line):
    """
    Assume lines of the form: "ax:1234, ay:1230, az:1010"
    Return raw integer for ax channel.
    """
    try:
        text = line.decode("ascii").strip()
        parts = text.split(",")
        first = parts[0].split(":")
        return int(first[1])
    except Exception:
        return None

def acquire_raw_samples(ser, n_samples):
    raw = []
    while len(raw) != n_samples:
        line = ser.readline()
        if not line:
            continue
        code = parse_line(line)
        if code is None:
            continue
        raw.append(code)
    return np.array(raw, dtype=float)

if __name__ == "__main__":
    ser = open_serial(PORT, BAUD)
    try:
        print("Collecting {} samples...".format(N_SAMPLES))
        raw = acquire_raw_samples(ser, N_SAMPLES)
    finally:
        ser.close()

    # Convert to engineering units
    w = a_hat * raw + b_hat

    mean_w = np.mean(w)
    std_w = np.std(w, ddof=1)

    print("Mean (units):", mean_w)
    print("Std dev (units):", std_w)

    # Check against expected static reference, e.g., 1 g on this axis
    w_star = 9.81  # example
    offset = mean_w - w_star
    print("Offset from expected (units):", offset)
      

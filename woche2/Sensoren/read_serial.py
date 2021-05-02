import io

import pynmea2
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
file_suffix = time.time()

with open(f"data_{file_suffix}.txt", "w") as f:
    f.write(f"lat,lon\n")

msg_count = 0

while msg_count < 1000:
    try:
        line = sio.readline()
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.GLL):
            with open(f"data_{file_suffix}.txt", "a") as f:
                f.write(f"{msg.lat},{msg.lon}\n")
                msg_count += 1
    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        continue
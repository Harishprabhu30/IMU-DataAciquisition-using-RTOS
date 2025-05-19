import serial
import matplotlib.pyplot as plt
import numpy as np
import time

# ============== USER-CONFIGURABLE PARAMETERS ==============
SERIAL_PORT = '/dev/tty.usbmodem14201'  # Change to your actual port
BAUD_RATE = 115200
SAMPLES_TO_COLLECT = 3000               # Match with MAX_SAMPLES in firmware
EXPECTED_SAMPLING_RATE = 100            # Hz — match with firmware
LABEL = 'STAIRUP_1'                     # Activity label
# ===========================================================

samples = []

print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=90) as ser:
    print("Connected. Flushing residual data...")
    for _ in range(3):
        ser.flushInput()
        time.sleep(1)

    print("Reading data...")
    while len(samples) < SAMPLES_TO_COLLECT:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if "Transfer complete" in line:
                print("Transfer complete message received. Ending read.")
                break

            parts = line.split(',')
            if len(parts) != 7:
                print(f"Skipping malformed line: {line}")
                continue

            ax, ay, az, gx, gy, gz, timestamp = map(float, parts)
            if abs(ax) < 100 and abs(ay) < 100 and abs(az) < 100:  # Relaxed sanity check
                samples.append((ax, ay, az, gx, gy, gz, int(timestamp)))
                print(f"Read sample {len(samples)}: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}, "
                      f"gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}, ts={timestamp}")

        except Exception as e:
            print(f"Error parsing line: {e}")
            continue

if len(samples) < SAMPLES_TO_COLLECT:
    print(f"WARNING: Only {len(samples)} samples collected. Expected {SAMPLES_TO_COLLECT}.")

# Convert to NumPy structured array
imu_array = np.array([(d[0], d[1], d[2], d[3], d[4], d[5], d[6], LABEL) for d in samples],
                     dtype=[('ax', float), ('ay', float), ('az', float),
                            ('gx', float), ('gy', float), ('gz', float),
                            ('timestamp', np.int64), ('label', 'U32')])
print(f"\nData stored in imu_array (NumPy) with {len(imu_array)} samples.")
print(imu_array)

# Estimate actual sampling rate
timestamps = imu_array['timestamp'] / 1_000_000.0  # Convert µs to seconds
time_diffs = np.diff(timestamps)
actual_sampling_rate = 1.0 / np.mean(time_diffs) if len(time_diffs) > 0 else EXPECTED_SAMPLING_RATE
print(f"Estimated Sampling Rate: {actual_sampling_rate:.2f} Hz")

# Time vector (relative)
relative_time = timestamps - timestamps[0]

# ================== TIME DOMAIN PLOT ==================
plt.figure(figsize=(12, 5))

# Accelerometer
plt.subplot(2, 1, 1)
plt.plot(relative_time, imu_array['ax'], label='AX (g)', color='r')
plt.plot(relative_time, imu_array['ay'], label='AY (g)', color='g')
plt.plot(relative_time, imu_array['az'], label='AZ (g)', color='b')
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (g)")
plt.title(f"Accelerometer Data ({LABEL}, Fs = {int(actual_sampling_rate)}Hz)")
plt.legend()
plt.grid(True)

# Gyroscope
plt.subplot(2, 1, 2)
plt.plot(relative_time, imu_array['gx'], label='GX (deg/s)', color='r')
plt.plot(relative_time, imu_array['gy'], label='GY (deg/s)', color='g')
plt.plot(relative_time, imu_array['gz'], label='GZ (deg/s)', color='b')
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (deg/s)")
plt.title(f"Gyroscope Data ({LABEL}, Fs = {int(actual_sampling_rate)}Hz)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig(f'{LABEL.lower()}.png')
print(f"Plot saved as '{LABEL.lower()}.png'")
plt.show()

import serial
import struct
import numpy as np

# Serial port configuration
port = '/dev/tty.usbmodem14201'  # Adjust to your port (use ls /dev/cu.*)
ser = serial.Serial(port, 115200, timeout=1)

# Data storage (list for STAIRUP_1, extensible for other labels)
imu_data_stairup_1 = []  # List for STAIRUP_1 samples

print("Listening for IMU data (label: STAIRUP_1)...")
try:
    while True:
        # Read imu_sample_t (6 floats + 1 int64_t = 32 bytes)
        sample_bytes = ser.read(32)
        if len(sample_bytes) == 32:
            ax, ay, az, gx, gy, gz, timestamp = struct.unpack('<ffffffq', sample_bytes)
            imu_data_stairup_1.append((ax, ay, az, gx, gy, gz, timestamp))
            print(f"STAIRUP_1: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}, gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}, ts={timestamp}")

except KeyboardInterrupt:
    ser.close()
    print(f"Stopped. Received {len(imu_data_stairup_1)} samples for STAIRUP_1.")

# Convert to NumPy array
imu_array_stairup_1 = np.array([(d[0], d[1], d[2], d[3], d[4], d[5], d[6], 'STAIRUP_1') for d in imu_data_stairup_1],
                              dtype=[('ax', float), ('ay', float), ('az', float), ('gx', float), ('gy', float), ('gz', float), ('timestamp', np.int64), ('label', 'U32')])
print("Data stored in imu_array_stairup_1 (NumPy):")
print(imu_array_stairup_1)

# Optional: Extend for other labels
# imu_data_stairdown_1 = []
# Run the script again with a different label assignment